#!/usr/bin/env python3
# Copyright 2026 Ângelo Moisés Alves
# Licensed under the MIT license, see the LICENSE file for details.
"""On-hardware self-test for PicoENC28J60.

Runs against a Pico flashed with extras/bench/BenchTest and wired to an
ENC28J60 that is cabled to the same LAN as this PC. It generates real traffic
(ping, UDP broadcast, UDP unicast) and checks what the board saw and sent.

    python3 extras/bench/bench.py --ip 192.168.3.77 --flash
    python3 extras/bench/bench.py --ip 192.168.3.77 --flash --irq 6    # INT wired to GP6
    python3 extras/bench/bench.py --ip 192.168.3.77 --flash --profile  # the build with the counters; adds cost

With --irq the board is built to be driven by the chip's INT pin: the main loop
talks to the chip only when the pin asks for it. All the tests above then run on
top of that, and three more (irqidle, irqedge, irqlink) check the interrupt itself.

Besides the traffic, the tests check what the driver does when things go wrong: a
begin() with no chip, with settings that make no sense and with no room in the PIO,
other users of the PIO, a state machine that stops, begin() after end(), and the
status of a transmitted frame. Others go through the rest of the public API: the
functions that only look and discardFrame() on frames that wait in the buffer,
hasLinkChange(), setMACAddress() and the interrupt callback.

Two tests measure the driver instead of checking a behavior. txfull sends 200 full-size frames back to back and reports
the frames and the kB a second that the driver reaches. cost counts the commands of the SPI that available(),
receiveFrame(), an upload and the start of a transmission take, and fails if one takes more than the code needs. The C
driver counts only when it is compiled with -DENC28J60_PROFILE, which is not the build that users get: --flash alone
builds BenchTest as before, --flash --profile builds it with the counters, and cost runs only with --profile, the way the
interrupt tests run only with --irq.

The C driver also has asynchronous functions (DMA with notification callbacks) that only the FreeRTOS+TCP driver
uses. Seven more tests go through them: the board hands the chip over (its ASYNC command) and serves it the way that
driver does. asyncrx and asyncdiscard read ERXRDPT back after every frame and want it odd; asyncfast and asyncecho
call the next function with nothing in between; asynctx sends through the upload slot and the transmit interrupt;
asyncflood overloads the receiver; asynccycle starts and stops the driver over and over to see that it gives back its
DMA channels and its memory. It runs last, because a driver that keeps the channels runs out of them and stops the
firmware.

Needs pyserial, arduino-cli (only with --flash) and a free IPv4 address on the
LAN for the board, and a second one for the setmac test (--alt-ip, by default the
address of the board plus two). No root required.
"""
import argparse
import glob
import os
import re
import socket
import struct
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from deploy import deploy  # noqa: E402

try:
    import serial
except ImportError:
    sys.exit("pyserial is required: pip install pyserial")

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SKETCH = os.path.join(REPO, "extras", "bench", "BenchTest")
PICO_MAC = "02:50:49:43:4f:01"
PORT_RX, PORT_ECHO, PORT_TX, PEER_PORT = 9999, 9997, 9998, 9998
SIZES = [10, 18, 100, 400, 800, 1200, 1472]  # UDP payload sizes
ALL_TESTS = ["init", "cfgerr", "pioful", "spi", "ping", "rx", "echo", "tx", "txlong", "txburst", "txb", "txblong",
             "txbburst", "txfull", "soak", "rxflood", "rxoverflow", "discard", "callback", "linkflag", "setmac",
             "txstat", "pioshare", "stall", "lifecycle", "txmix", "txring", "lostchip", "asyncrx", "asyncdiscard",
             "asyncfast", "asyncecho", "asynctx", "asyncflood", "asynccycle"]
ASYNC_TESTS = ["asyncrx", "asyncdiscard", "asyncfast", "asyncecho", "asynctx", "asyncflood"]  # one ASYNC session
DEFAULT_SPI_HZ = 10000000  # ENC28J60Config::spi_hz
SPI_SPEEDS = [1000000, 3500000, 8000000, 10000000, 12500000, 16000000, 20000000]  # Hz, for the spi test
IRQ_TESTS = ["irqidle", "irqedge", "irqlink"]  # only with --irq
PROFILE_TESTS = ["cost"]  # only with --profile: they read the counters of a driver built with -DENC28J60_PROFILE
results = []


# --- payload pattern (must match BenchTest.ino) -----------------------------
def fill(seq, i):
    return (seq * 131 + i * 7 + 13) & 0xFF


def make_payload(seq, plen):
    b = bytearray(plen)
    b[0:4] = b"PBNC"
    b[4:8] = struct.pack(">I", seq)
    b[8:10] = struct.pack(">H", plen)
    for i in range(10, plen):
        b[i] = fill(seq, i)
    return bytes(b)


def check_payload(data):
    """Return the sequence number, or None if the datagram is malformed."""
    if len(data) < 10 or data[0:4] != b"PBNC":
        return None
    seq = struct.unpack(">I", data[4:8])[0]
    if struct.unpack(">H", data[8:10])[0] != len(data):
        return None
    for i in range(10, len(data)):
        if data[i] != fill(seq, i):
            return None
    return seq


# --- serial link to the board -----------------------------------------------
class Pico:
    def __init__(self, port):
        self.s = serial.Serial(port, 115200, timeout=0.2)
        self.rx = b""

    def _readline(self, deadline):
        while time.time() < deadline:
            i = self.rx.find(b"\n")
            if i >= 0:
                line, self.rx = self.rx[:i], self.rx[i + 1:]
                return line.decode("utf-8", "replace").strip()
            self.rx += self.s.read(256)
        return None

    def send(self, text):
        self.s.write((text + "\n").encode())

    def expect(self, prefix, timeout=5.0, skipped=None):
        """Wait for a line that starts with prefix (or any of a tuple of them); `skipped`, if given, collects the others."""
        deadline = time.time() + timeout
        while True:
            line = self._readline(deadline)
            if line is None:
                raise TimeoutError("no '%s' reply within %ss" % (prefix, timeout))
            if line.startswith(prefix):
                return line
            if skipped is not None:
                skipped.append(line)

    def cmd(self, text, prefix, timeout=5.0):
        self.s.reset_input_buffer()
        self.rx = b""
        self.send(text)
        return self.expect(prefix, timeout)


def kv(line):
    out = {}
    for tok in line.split()[1:]:
        if "=" in tok:
            k, v = tok.split("=", 1)
            out[k] = v
    return out


def num(d, k):
    return int(d[k], 0)


def record(name, ok, detail):
    results.append(ok)
    print("[%s] %-8s %s" % ("PASS" if ok else "FAIL", name, detail))


def info(name, detail):
    print("[INFO] %-8s %s" % (name, detail))


# --- setup helpers ----------------------------------------------------------
def find_port(explicit):
    if explicit:
        return explicit
    cands = sorted(glob.glob("/dev/serial/by-id/*Pico*"))
    if not cands:
        sys.exit("no Pico serial port found; pass --port")
    # --flash writes over whatever runs on that board: with several Picos connected, picking one would be a guess
    if len(cands) > 1:
        sys.exit("%d Picos are connected (%s): pass --port with the one wired to the ENC28J60, so that no other "
                 "board is flashed" % (len(cands), ", ".join(os.path.basename(c) for c in cands)))
    return cands[0]


def flash(port, fqbn, irq_pin=None, sketch=SKETCH, spi_hz=None, profile=False):
    """Compile a bench sketch against this checkout and upload it. With profile the C driver and the sketch are built with
    -DENC28J60_PROFILE, so that the driver counts its commands and BenchTest can report them (COST)."""
    variant = (("irq%d" % irq_pin if irq_pin is not None else "poll") + ("-%dhz" % spi_hz if spi_hz else "") +
               ("-prof" if profile else ""))
    build = os.path.join(tempfile.gettempdir(), "picoenc-%s-build-%s" % (os.path.basename(sketch), variant))
    cmd = ["arduino-cli", "compile", "--fqbn", fqbn, "--library", REPO, "--build-path", build]
    defines = []
    if profile:
        # The driver is C and the sketch is C++: the define has to reach both
        defines.append("-DENC28J60_PROFILE")
        cmd += ["--build-property", "compiler.c.extra_flags=-DENC28J60_PROFILE"]
    if irq_pin is not None:
        defines.append("-DBENCH_IRQ_PIN=%d" % irq_pin)
    if spi_hz:
        defines.append("-DBENCH_SPI_HZ=%d" % spi_hz)
    if defines:
        cmd += ["--build-property", "compiler.cpp.extra_flags=" + " ".join(defines)]
    subprocess.run(cmd + [sketch], check=True)
    # Not the uploader of the core, which writes the image to every board in BOOTSEL: deploy.py writes it only to the
    # one on the USB port of this serial port
    deploy(port, os.path.join(build, os.path.basename(sketch) + ".ino.uf2"))
    # Wait for the board to drop off the bus and come back with the new firmware.
    deadline = time.time() + 5
    while os.path.exists(port) and time.time() < deadline:
        time.sleep(0.2)
    deadline = time.time() + 30
    while not os.path.exists(port):
        if time.time() > deadline:
            sys.exit("serial port did not come back after upload")
        time.sleep(0.3)
    time.sleep(2.0)


def local_ip_for(dst):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((dst, 9))
        return s.getsockname()[0]
    finally:
        s.close()


def stats(p):
    return kv(p.cmd("STAT", "STAT"))


# --- tests ------------------------------------------------------------------
def t_init(p, link_timeout=20):
    d = kv(p.cmd("INFO", "INFO"))
    rev = num(d, "rev")
    record("init", num(d, "init") == 1 and rev in (2, 4, 5, 6),
           "initialised=%s revision=0x%02X mac=%s irq pin=%s" % (d["init"], rev, d["mac"], d.get("irqpin", "?")))
    record("errpath", num(d, "probe_ok") == 0 and d.get("probe_err") == "nochip" and num(d, "probe_ms") < 500 and
           num(d, "init") == 1,
           "begin() with the CS pin on an unconnected GPIO %s (%s) after %s ms; the real begin() right after worked"
           % ("failed" if num(d, "probe_ok") == 0 else "SUCCEEDED (wrong)", d.get("probe_err"), d["probe_ms"]))
    deadline = time.time() + link_timeout
    link = 0
    while time.time() < deadline:
        link = num(kv(p.cmd("INFO", "INFO")), "link")
        if link:
            break
        time.sleep(0.5)
    record("link", link == 1, "up" if link else "down after %ss (cable? 10BASE-T port?)" % link_timeout)
    return link == 1


def t_ping(p, ip, count=20):
    p.cmd("IP " + ip, "OK")
    p.cmd("RESET", "OK")
    r = subprocess.run(["ping", "-c", str(count), "-i", "0.2", "-W", "1", ip],
                       capture_output=True, text=True)
    m = re.search(r"(\d+) packets transmitted, (\d+) received", r.stdout)
    sent, got = (int(m.group(1)), int(m.group(2))) if m else (count, 0)
    neigh = subprocess.run(["ip", "neigh", "show", ip], capture_output=True, text=True).stdout.strip()
    s = stats(p)
    record("ping", got == sent and PICO_MAC in neigh.lower(),
           "%d/%d replies; host ARP entry: %s; board: arp=%s icmp=%s txf=%s" %
           (got, sent, neigh or "none", s["arp"], s["icmp"], s["txf"]))


def t_rx(p, host_ip, count=200, interval=0.01):
    p.cmd("RESET", "OK")
    sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sk.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sk.bind((host_ip, 0))
    for seq in range(count):
        plen = SIZES[seq % len(SIZES)]
        sk.sendto(make_payload(seq, plen), ("255.255.255.255", PORT_RX))
        time.sleep(max(interval, (plen + 46) * 8 / 10e6 * 4))
    sk.close()
    time.sleep(0.5)
    s = stats(p)
    good = (num(s, "bench") == count and num(s, "dup") == 0 and num(s, "stale") == 0 and
            num(s, "gap") == 0 and num(s, "bad") == 0 and num(s, "peekbad") == 0)
    record("rx", good,
           "sent %d broadcast frames; board: ok=%s dup=%s stale=%s gap=%s bad=%s; other LAN frames seen=%s; "
           "bytes after IP datagram: 0B x%s, 4B(FCS) x%s, other x%s" %
           (count, s["bench"], s["dup"], s["stale"], s["gap"], s["bad"], s["frames"],
            s["x0"], s["x4"], s["xo"]))


def stream_rx(p, host_ip, count, interval, sizes, pace=True):
    sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sk.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sk.bind((host_ip, 0))
    for seq in range(count):
        plen = sizes[seq % len(sizes)]
        sk.sendto(make_payload(seq, plen), ("255.255.255.255", PORT_RX))
        gap = max(interval, (plen + 46) * 8 / 10e6 * 4) if pace else interval
        if gap:
            time.sleep(gap)
    sk.close()
    time.sleep(0.7)
    return stats(p)


def wait_quiet(p, max_s=30):
    """Wait until the board stops receiving: whatever the switch had queued has been delivered."""
    last, still = -1, 0
    deadline = time.time() + max_s
    while time.time() < deadline and still < 3:
        now = num(stats(p), "frames")
        still = still + 1 if now == last else 0
        last = now
        time.sleep(0.4)


def t_soak(p, host_ip, count=5000):
    """A long RX stream at a rate the chip can sustain: nothing may repeat, reorder, get lost or need a recovery."""
    p.cmd("RESET", "OK")
    t0 = time.time()
    s = stream_rx(p, host_ip, count, 0.002, [18, 100, 400, 800, 1200, 1472])
    good = (num(s, "bench") == count and num(s, "dup") == 0 and num(s, "stale") == 0 and
            num(s, "gap") == 0 and num(s, "bad") == 0 and num(s, "rxz") == 0 and num(s, "runt") == 0 and
            num(s, "peekbad") == 0)
    record("soak", good, "%d frames in %.0f s: ok=%s dup=%s stale=%s gap=%s bad=%s runt=%s recoveries=%s; "
           "ambient frames=%d" % (count, time.time() - t0, s["bench"], s["dup"], s["stale"], s["gap"], s["bad"],
                                  s["runt"], s["rxz"], num(s, "frames") - num(s, "bench")))


def t_rxflood(p, host_ip, count=3000, name="rxflood", board_async=False):
    """Overload the receiver on purpose. Frames may be lost, but what arrives must be intact and the
    receiver must work normally afterwards. With board_async (asyncflood) the board is in its asynchronous mode, where
    no frame header may make no sense and no wait for the DMA may time out either."""
    p.cmd("RESET", "OK")
    # Big frames, no pacing: they reach the board back to back at the 10 Mbit/s of
    # its port. At the default SPI clock the chip cannot drain its receive buffer as
    # fast, and overflows; at a clock of 16 MHz or more it keeps up with the wire and
    # the switch drops the frames before they get there. Either way what arrives has to
    # be intact, and the receiver has to be in step afterwards (rxoverflow overflows the
    # buffer on purpose, whatever the clock).
    s = stream_rx(p, host_ip, count, 0, [1472, 1200, 800], pace=False)
    intact = num(s, "dup") == 0 and num(s, "stale") == 0 and num(s, "bad") == 0 and num(s, "peekbad") == 0
    got, lost = num(s, "bench"), count - num(s, "bench")
    ok1 = intact and got > 0
    wait_quiet(p)  # frames queued in the switch keep arriving for a while after the flood
    flooded = stats(p) if board_async else s  # the flood has been served by now
    p.cmd("RESET", "OK")
    s2 = stream_rx(p, host_ip, 50, 0.01, [100])
    ok2 = (num(s2, "bench") == 50 and num(s2, "gap") == 0 and num(s2, "bad") == 0 and
           num(s2, "dup") == 0 and num(s2, "stale") == 0)
    sound, tail = True, ""
    if board_async:
        # A frame that the driver never takes stays in the buffer of the chip: none may be left after the clean stream
        sound = all(num(x, k) == 0 for x in (flooded, s2) for k in ("hdrbad", "waitto")) and num(s2, "pend") == 0
        tail = "; bad headers %s and %s, wait timeouts %s and %s (flood, clean stream), frames left in the buffer %s" % (
            flooded["hdrbad"], s2["hdrbad"], flooded["waitto"], s2["waitto"], s2["pend"])
    record(name, ok1 and ok2 and sound, "flood of %d frames: %d received intact, %d lost (the buffer of the chip "
           "overflowed in %s of the service passes), dup=%s stale=%s bad=%s, recoveries=%s; clean stream afterwards: "
           "%s/50 (gap=%s)%s" %
           (count, got, lost, s["rxe" if board_async else "peekrxe"], s["dup"], s["stale"], s["bad"], s["rxz"],
            s2["bench"], s2["gap"], tail))


def t_rxoverflow(p, host_ip, irq, count=12, plen=1472):
    """The receive buffer overflows on purpose, at any SPI clock: the board does not serve the chip while full-size
    frames come in. The oldest frames stay, the rest are dropped by the chip, the error is reported by
    hasRXError() and cleared by the acknowledge, and the receiver is in step afterwards."""
    p.cmd("RESET", "OK")
    p.cmd("HOLD 1", "OK")
    sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sk.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sk.bind((host_ip, 0))
    for seq in range(count):
        sk.sendto(make_payload(seq, plen), ("255.255.255.255", PORT_RX))
        time.sleep(0.02)  # 1.2 ms on the wire each: the default receive buffer holds three of them
    sk.close()
    time.sleep(0.3)
    before = kv(p.cmd("PEEK", "PEEK"))
    line = num(stats(p), "int")
    p.cmd("HOLD 0", "OK")
    time.sleep(1.0)
    s = stats(p)
    after = kv(p.cmd("PEEK", "PEEK"))
    line_after = num(s, "int")
    p.cmd("RESET", "OK")
    s2 = stream_rx(p, host_ip, 50, 0.01, [100])
    held, flags = num(before, "avail"), num(before, "flags")
    ok = (flags & 0x41 == 0x41 and 2 <= held < count and (not irq or line == 0) and
          1 <= num(s, "bench") <= held and num(s, "gap") == 0 and num(s, "bad") == 0 and num(s, "dup") == 0 and
          num(s, "stale") == 0 and num(s, "rxz") == 0 and num(s, "peekbad") == 0 and
          num(s, "peekrxe") >= 1 and num(s, "rxe") >= 1 and
          num(after, "pkt") == 0 and not num(after, "flags") & 0x01 and (not irq or line_after == 1) and
          num(s2, "bench") == 50 and num(s2, "gap") == 0 and num(s2, "bad") == 0)
    record("rxoverflow", ok,
           "%d full-size frames sent to a board that does not serve the chip: it held %d frames, flags 0x%02X%s; "
           "after serving it %s of the %d bench frames were received intact (gap=%s bad=%s dup=%s), the error was "
           "reported %s times and acknowledged (flags 0x%02X)%s; clean stream %s/50" %
           (count, held, flags, ", INT line %s" % ("low" if line == 0 else "HIGH") if irq else "",
            s["bench"], count, s["gap"], s["bad"], s["dup"], s["peekrxe"], num(after, "flags"),
            ", INT line %s" % ("high" if line_after == 1 else "LOW") if irq else "", s2["bench"]))


def run_echo(p, host_ip, pico_ip, count):
    """`count` unicast UDP round trips PC -> board -> PC. Returns lost, corrupted, average rtt in ms and the board's STAT."""
    p.cmd("RESET", "OK")
    sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sk.bind((host_ip, PEER_PORT))
    sk.settimeout(0.5)
    lost = bad = 0
    rtts = []
    for seq in range(count):
        plen = SIZES[seq % len(SIZES)]
        data = make_payload(seq, plen)
        t0 = time.time()
        sk.sendto(data, (pico_ip, PORT_ECHO))
        try:
            got, _ = sk.recvfrom(2048)
            rtts.append(time.time() - t0)
            if got != data:
                bad += 1
        except socket.timeout:
            lost += 1
    sk.close()
    s = stats(p)
    avg = 1000 * sum(rtts) / len(rtts) if rtts else float("nan")
    return lost, bad, avg, s


def t_echo(p, host_ip, pico_ip, count=200):
    lost, bad, avg, s = run_echo(p, host_ip, pico_ip, count)
    record("echo", lost == 0 and bad == 0,
           "%d unicast round trips: lost=%d corrupted=%d avg rtt=%.1f ms; board: echo=%s txf=%s" %
           (count, lost, bad, avg, s["echo"], s["txf"]))


def run_tx(p, host_ip, pico_ip, count, plen, gap_us):
    """Make the board learn us as peer, then have it stream `count` unicast frames to us."""
    p.cmd("RESET", "OK")
    sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sk.bind((host_ip, PEER_PORT))
    sk.settimeout(0.5)
    sk.sendto(make_payload(0, 18), (pico_ip, PORT_ECHO))
    try:
        sk.recvfrom(2048)  # the echo that makes the board remember us
    except socket.timeout:
        sk.close()
        raise RuntimeError("no echo from the board; cannot start the TX stream")
    p.send("TX %d %d %d" % (count, plen, gap_us))
    got, dup, corrupt = set(), 0, 0
    sk.settimeout(1.5)
    while len(got) < count:
        try:
            data, _ = sk.recvfrom(2048)
        except socket.timeout:
            break
        seq = check_payload(data)
        if seq is None:
            corrupt += 1
        elif seq in got:
            dup += 1
        else:
            got.add(seq)
    sk.close()
    done = kv(p.expect("TXDONE", 10))
    return len(got), dup, corrupt, num(done, "sent"), num(done, "fail")


def run_txb(p, host_ip, count, plen, gap_us, done=None):
    """Have the board broadcast `count` frames; needs no RX path on the board. `done`, if given, is filled with the
    fields of the TXDONE line (sent, fail and us, the time of the whole stream)."""
    p.cmd("RESET", "OK")
    sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sk.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sk.bind(("", PORT_TX))
    p.send("TXB %d %d %d" % (count, plen, gap_us))
    got, dup, corrupt = set(), 0, 0
    sk.settimeout(1.5)
    while len(got) < count:
        try:
            data, _ = sk.recvfrom(2048)
        except socket.timeout:
            break
        seq = check_payload(data)
        if seq is None:
            corrupt += 1
        elif seq in got:
            dup += 1
        else:
            got.add(seq)
    sk.close()
    line = kv(p.expect("TXDONE", 10))
    if done is not None:
        done.update(line)
    return len(got), dup, corrupt, num(line, "sent"), num(line, "fail")


def t_txb(p, host_ip, name, count, plen, gap_us):
    got, dup, corrupt, sent, fail = run_txb(p, host_ip, count, plen, gap_us)
    record(name, got == count and dup == 0 and corrupt == 0 and fail == 0,
           "board sendFrame ok=%d failed=%d (%d B payload, %d us gap, broadcast); host received %d/%d, "
           "duplicates=%d corrupted=%d" % (sent, fail, plen, gap_us, got, count, dup, corrupt))


def t_txfull(p, host_ip, count=200, plen=1472):
    """The longest frames that the stream can carry, one after the other with no gap: every one has to arrive intact. The
    time of the whole stream, which the board reports in TXDONE (us=), is the speed: frames, and kB of UDP payload, a
    second. It is reported, not judged: it depends on the SPI clock of the build."""
    done = {}
    got, dup, corrupt, sent, fail = run_txb(p, host_ip, count, plen, 0, done)
    us = int(done.get("us", 0))
    if us <= 0:
        record("txfull", False, "TXDONE has no us= (a BenchTest from before the timing: use --flash); sent=%s fail=%s" %
               (done.get("sent"), done.get("fail")))
        return
    seconds = us / 1e6
    record("txfull", got == count and dup == 0 and corrupt == 0 and fail == 0,
           "board sendFrame ok=%d failed=%d (%d B payload, no gap, broadcast) in %.1f ms: %.0f frames/s, %.0f kB/s of UDP "
           "payload (1 kB = 1000 bytes), %.0f us a frame; host received %d/%d, duplicates=%d corrupted=%d" %
           (sent, fail, plen, us / 1000.0, sent / seconds, sent * plen / seconds / 1000.0, us / max(sent, 1), got, count,
            dup, corrupt))


def t_tx(p, host_ip, pico_ip, name, count, plen, gap_us):
    got, dup, corrupt, sent, fail = run_tx(p, host_ip, pico_ip, count, plen, gap_us)
    record(name, got == count and dup == 0 and corrupt == 0 and fail == 0,
           "board sendFrame ok=%d failed=%d (%d B payload, %d us gap); host received %d/%d, "
           "duplicates=%d corrupted=%d" % (sent, fail, plen, gap_us, got, count, dup, corrupt))


def t_cfgerr(p):
    """Settings that make no sense are refused at once, as such, without the pins being touched."""
    d = kv(p.cmd("INFO", "INFO"))
    tried, rejected = num(d, "cfg_tried"), num(d, "cfg_rejected")
    record("cfgerr", tried >= 7 and rejected == tried and num(d, "cfg_us") < 5000,
           "%d of %d invalid configurations refused with 'config', the slowest in %s us" % (rejected, tried, d["cfg_us"]))


def t_pioful(p):
    """With no room in any PIO, begin() says so, quickly, and the real begin() afterwards still works."""
    d = kv(p.cmd("INFO", "INFO"))
    record("pioful", num(d, "pioful_filled") == 1 and num(d, "pioful_ok") == 0 and d.get("pioful_err") == "piospace" and
           num(d, "pioful_ms") < 100 and num(d, "init") == 1,
           "instruction memory of every PIO full: begin() failed with '%s' after %s ms; the real begin() right after "
           "worked (pio%s sm%s, claimed=%s)" % (d.get("pioful_err"), d["pioful_ms"], d["pio"], d["sm"], d["claimed"]))


def ping_replies(ip, count=5):
    r = subprocess.run(["ping", "-c", str(count), "-i", "0.2", "-W", "1", ip], capture_output=True, text=True)
    m = re.search(r"(\d+) packets transmitted, (\d+) received", r.stdout)
    return int(m.group(2)) if m else 0


def wait_link(p, timeout=25):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if num(kv(p.cmd("INFO", "INFO")), "link"):
            return True
        time.sleep(0.5)
    return False


def t_spi(p, ip, assert_up_to, expect_hz, rounds=2000):
    """Data through every kind of access at several SPI clocks. What the wiring can carry is not up to the
    library, so only the clocks up to --spi-assert-mhz have to be clean; the faster ones are reported."""
    ok, notes, timing = True, [], []
    default_hz = num(kv(p.cmd("INFO", "INFO")), "spihz")
    for hz in SPI_SPEEDS:
        t = kv(p.cmd("SPITEST %d %d" % (hz, rounds), "SPITEST", 90))
        errors = sum(num(t, k) for k in ("eth", "mac", "mii", "mem"))
        clean = num(t, "began") == 1 and errors == 0 and num(t, "phid1") == 0x0083 and num(t, "io") == 0
        # The divider of the PIO is fractional: one SPI period (4 PIO clocks) lasts floor(4 x div) or ceil(4 x div)
        # system clocks. The shortest one has to respect the 50 ns of the data sheet (20 MHz) whatever was asked for:
        # that is up to the driver, not to the wiring, so it is checked at every clock.
        four = 4 * (num(t, "div_int") + num(t, "div_frac") / 256.0)
        sys_hz = num(t, "sys_hz")
        shortest_ns = int(four) * 1e9 / sys_hz if sys_hz else 0
        in_spec = num(t, "began") == 1 and shortest_ns >= 50
        timing.append("%.2f->%.2f MHz %.0f ns" % (hz / 1e6, sys_hz / four / 1e6 if four else 0, shortest_ns))
        ok = ok and in_spec
        line = "%5.2f MHz: %s; frame read %.2f ms (49 x 31 bytes), register read %s us" % (
            hz / 1e6, "clean" if clean else ("did not start (%s)" % t.get("err") if num(t, "began") == 0 else
                                              "%d errors (eth %s, mac %s, mii %s, memory %s)" % (
                                                  errors, t["eth"], t["mac"], t["mii"], t["mem"])),
            num(t, "frame_us") / 1000.0, t["reg_us"])
        if hz <= assert_up_to:
            ok = ok and clean and num(t, "restored") == 1
            notes.append(line)
        else:
            info("spi", line + " (above --spi-assert-mhz: not a failure)")
    notes.append("asked -> effective clock and shortest period (50 ns at least): " + ", ".join(timing))
    link = wait_link(p)
    replies = ping_replies(ip) if link else 0
    ok = ok and link and replies == 5 and default_hz == expect_hz
    record("spi", ok, "default clock %.1f MHz; %s; link %s, ping %d/5" %
           (default_hz / 1e6, "; ".join(notes), "up" if link else "DOWN", replies))


def t_discard(p, host_ip, irq, count=9):
    """Frames wait in the receive buffer while the board does not serve the chip. hasPacket(), available() and
    peekInterruptFlags() have to say so, discardFrame() has to drop one frame unseen without disturbing the
    next ones, and the receiver has to be in step afterwards."""
    p.cmd("RESET", "OK")
    p.cmd("HOLD 1", "OK")
    sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sk.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sk.bind((host_ip, 0))
    for seq in range(count):
        sk.sendto(make_payload(seq, 100), ("255.255.255.255", PORT_RX))
        time.sleep(0.03)
    sk.close()
    time.sleep(0.3)
    before = kv(p.cmd("PEEK", "PEEK"))
    line_before = num(stats(p), "int")
    # DRAIN also says what hasPacket(), available() and peekInterruptFlags() return right after it: asked by a
    # separate PEEK, a frame of the LAN that came in meanwhile made them look wrong
    drained = after = kv(p.cmd("DRAIN", "DRAINDONE", 10))
    p.cmd("HOLD 0", "OK")
    time.sleep(0.5)
    s = stats(p)
    total = num(drained, "total")
    expect_dropped = sum(1 for i in range(total) if i % 3 == 1)
    dropped = num(drained, "dropped")
    p.cmd("RESET", "OK")
    s2 = stream_rx(p, host_ip, 50, 0.01, [100])  # the pointers must still be in step
    ok = (num(before, "pkt") == 1 and num(before, "avail") >= count and num(before, "flags") & 0x40 and
          (not irq or line_before == 0) and
          total >= count and dropped == expect_dropped and dropped >= 3 and num(drained, "left") == 0 and
          num(after, "pkt") == 0 and num(after, "avail") == 0 and not num(after, "flags") & 0x40 and
          num(s, "dis") == dropped and num(s, "bad") == 0 and num(s, "dup") == 0 and num(s, "stale") == 0 and
          num(s, "rxz") == 0 and count - dropped <= num(s, "bench") <= count and num(s, "peekbad") == 0 and
          num(s2, "bench") == 50 and num(s2, "gap") == 0 and num(s2, "bad") == 0)
    record("discard", ok,
           "%d frames held in the buffer: hasPacket()=%s, available()=%s, flags 0x%02X%s; drained %d frames, "
           "discardFrame() dropped %d unseen (every third); %s of the %d frames sent were received intact "
           "(bad=%s dup=%s stale=%s); afterwards hasPacket()=%s, available()=%s; clean stream %s/50" %
           (count, before["pkt"], before["avail"], num(before, "flags"),
            ", INT line %s" % ("low" if line_before == 0 else "HIGH") if irq else "", total, dropped, s["bench"],
            count, s["bad"], s["dup"], s["stale"], after["pkt"], after["avail"], s2["bench"]))


def t_callback(p, host_ip, irq, count=50, gap=0.05):
    """The interrupt callback of setInterruptCallback(): called from the interrupt handler when the INT pin is
    wired, by poll() when it is not. It only raises a flag; the main loop then serves the chip."""
    p.cmd("CBMODE 1", "OK")
    p.cmd("RESET", "OK")
    sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sk.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sk.bind((host_ip, 0))
    for seq in range(count):
        sk.sendto(make_payload(seq, 100), ("255.255.255.255", PORT_RX))
        time.sleep(gap)
    sk.close()
    time.sleep(0.5)
    s = stats(p)
    p.cmd("CBMODE 0", "OK")
    cb, edges, svc = num(s, "cb"), num(s, "irq"), num(s, "svc")
    # with the wire the callback is the interrupt handler: once per falling edge. Without it, once per pass of the
    # main loop that found something to do, and each such pass serves the chip.
    ties = abs(cb - edges) <= 1 if irq else abs(cb - svc) <= 1
    ok = (num(s, "bench") == count and num(s, "gap") == 0 and num(s, "bad") == 0 and num(s, "dup") == 0 and
          cb >= count * 0.9 and ties and num(s, "peekbad") == 0)
    record("callback", ok, "%d frames %d ms apart, served only when the callback asked: received %s; callback called "
           "%d times, %s; %s service passes" %
           (count, gap * 1000, s["bench"], cb, "%d edges on INT" % edges if irq else "once per poll() that found work",
            svc))


def t_linkflag(p, ping_ip):
    """hasLinkChange(): the PHY is powered down and up, and the flag has to be there each time, and go away when
    it is acknowledged. Not tied to the INT pin: it works by looking at the flags."""
    t = kv(p.cmd("LINKEVENT", "LINKEVENT", 40))
    link = wait_link(p)
    replies = ping_replies(ping_ip) if link else 0
    keys = ("clean", "down_seen", "link_down", "down_cleared", "up_seen", "link_up", "up_cleared")
    ok = all(num(t, k) == 1 for k in keys) and link and replies == 5
    record("linkflag", ok, "link change flag: %s; link %s, ping after the cycle %d/5" %
           (", ".join("%s=%s" % (k, t.get(k)) for k in keys), "up" if link else "DOWN", replies))


def t_setmac(p, ip, alt_ip):
    """setMACAddress() while the chip works: the chip's registers hold the new address, the board answers ARP with
    it and accepts frames sent to it, it survives the chip being reset, and the old address comes back."""
    alt_mac = "02:50:49:43:4f:02"
    notes, ok = [], True

    r = kv(p.cmd("SETMAC %s %s" % (alt_mac, alt_ip), "SETMAC"))
    replies = ping_replies(alt_ip)
    neigh = subprocess.run(["ip", "neigh", "show", alt_ip], capture_output=True, text=True).stdout.strip()
    good = num(r, "get") == 1 and num(r, "hw") == 1 and replies == 5 and alt_mac in neigh.lower()
    ok = ok and good
    notes.append("new address: registers %s, ping %d/5, host ARP entry %s" %
                 ("ok" if num(r, "hw") else "WRONG (%s)" % r.get("mac"), replies, neigh or "none"))

    t = kv(p.cmd("LOSTCHIP", "LOSTCHIP", 20))
    link = wait_link(p)
    replies = ping_replies(alt_ip) if link else 0
    good = num(t, "fixed") == 1 and link and replies == 5
    ok = ok and good
    notes.append("after the chip was reset: ping %d/5" % replies)

    r = kv(p.cmd("SETMAC %s %s" % (PICO_MAC, ip), "SETMAC"))
    replies = ping_replies(ip)
    good = num(r, "get") == 1 and num(r, "hw") == 1 and replies == 5
    ok = ok and good
    notes.append("original address back: ping %d/5" % replies)
    record("setmac", ok, "; ".join(notes))


def t_txstat(p):
    """The transmit status vector of frames sent on purpose: sizes, flags and no errors."""
    ok, notes = True, []
    # (frame length, broadcast, bytes the MAC reports: the frame padded to 60 bytes, plus the 4-byte CRC)
    for length, bcast, expect in ((60, 1, 64), (100, 1, 104), (100, 0, 104), (1514, 1, 1518), (40, 0, 64), (22, 1, 64)):
        t = kv(p.cmd("TSV %d %d" % (length, bcast), "TSV"))
        good = (t.get("status") == "0" and num(t, "retries") == 0 and num(t, "bytes") == expect and num(t, "wire") == expect
                and num(t, "done") == 1 and num(t, "bcast") == bcast and num(t, "mcast") == bcast and num(t, "late") == 0
                and num(t, "xcoll") == 0 and num(t, "underrun") == 0)
        ok = ok and good
        notes.append("%d%s->%s%s" % (length, "b" if bcast else "u", t.get("bytes"), "" if good else " WRONG(%s)" % t.get("raw")))
    record("txstat", ok, "status vectors, frame length -> bytes on the wire (b broadcast, u unicast): %s" % ", ".join(notes))


def ceil_div(a, b):
    return -(-a // b)


# The most commands that each step of COST may take, worked out by reading the code (src/PicoENC28J60.cpp,
# src/enc28j60-driver.c, src/enc28j60-cmd.h). Every register or buffer helper that ends in enc28j60_execute_blocking()
# is one command, except reg_write16_blk() (the low byte, then the high one), which is two, and bank_set_blk(), which is
# one for banks 0 and 3 (clear both BSEL bits, or set both) and two for banks 1 and 2 (clear both, then set one):
#   available() = enc28j60_rx_count_blk() = bank_set_blk(1) 2 + read EPKTCNT 1                                  = 3
#   receiveFrame() of a frame of rx_len bytes, which is what it returns:
#       _readRxInfo(): the packet count again 3 + the 6 bytes of header, one RBM 1                               = 4
#       enc28j60_frame_rx_blk(): the data in reads of RBM_MAX = 31 bytes at most                  ceil(rx_len / 31)
#           bank_set_blk(0) 1 + ERXRDPT 2 + ERDPT 2 + ECON2.PKTDEC 1                                             = 6
#       = 10 + ceil(rx_len / 31). The S of this formula is rx_len, not the byte count in the header of the chip: that
#       one includes the 4 bytes of the FCS, and receiveFrame() rewrites it as size - 4 before enc28j60_frame_rx_blk()
#       reads, so the reads are ceil((S_header - 4) / 31). For a frame of 62 bytes (66 in the header) they are 2, not 3.
#   enc28j60_frame_upload_blk() of up_len bytes: bank_set_blk(0) 1 + EWRPT 2 + the control byte, one WBM 1     = 4
#       then the frame in writes of WBM_MAX = 30 bytes at most                                    ceil(up_len / 30)
#       = 4 + ceil(up_len / 30)
#   enc28j60_frame_tx_blk(): bank_set_blk(0) 1 + set TXRST 1 + clear TXRST 1 + clear TXIF and TXERIF 1 + ETXST 2
#       + ETXND 2 + set TXRTS 1                                                                               = 9
# The wait after the start is not in the budget: it reads EIR until the frame is out, as many times as the wire takes,
# and clears TXRTS once, so its commands are eir + 1, which cost_problems() compares with the reads the driver counted.
def cost_budget(rx_len, up_len):
    return {"avail": 3, "rx": 10 + ceil_div(rx_len, 31), "up": 4 + ceil_div(up_len, 30), "start": 9}


def cost_over(d):
    """The steps of a COST line (as a dict) that took more commands than cost_budget() allows: (step, taken, allowed)."""
    budget = cost_budget(num(d, "rxlen"), num(d, "uplen"))
    return [(step, num(d, step), allowed) for step, allowed in budget.items() if num(d, step) > allowed]


def cost_problems(d):
    """What is wrong with a COST line, as sentences: a step over its budget, or a measurement not made as meant."""
    problems = ["%s took %d commands, the code needs %d" % over for over in cost_over(d)]
    if num(d, "rxlen") == 0:
        problems.append("no frame came in for receiveFrame()")
    if num(d, "empty") != 1:
        problems.append("the receive buffer was not empty for available()")
    if num(d, "sent") != 1:
        problems.append("the frame of the upload was not sent")
    if num(d, "eir") < 1 or num(d, "wait") != num(d, "eir") + 1:
        problems.append("the wait took %d commands for %d reads of EIR: it should be one more than the reads" %
                        (num(d, "wait"), num(d, "eir")))
    return problems


def cost_run(p, sk, seq, payload_len):
    """One COST. The board measures available() on an empty buffer and says COSTARM; the frame that it then takes is a
    UDP broadcast from this PC with a payload of payload_len bytes, and the board uploads and sends one as long.
    Returns the COST line, or the COSTERR line."""
    p.s.reset_input_buffer()
    p.rx = b""
    p.send("COST %d" % (42 + payload_len))
    line = p.expect(("COSTARM", "COSTERR"), 5)
    if line.startswith("COSTARM"):
        sk.sendto(make_payload(seq, payload_len), ("255.255.255.255", PORT_RX))
        line = p.expect(("COST ", "COSTERR"), 10)
    return line


def t_cost(p, host_ip, payloads=(18, 20, 400, 1472)):
    """The commands of the SPI that available() on an empty buffer, receiveFrame() of a frame, an upload and the start of
    a transmission take: the C driver counts them (the board is built with -DENC28J60_PROFILE, which --flash --profile
    does), and none may take more than cost_budget() says. The microseconds are reported, not judged."""
    # The suite of the gate runs this test alone, right after the board was flashed: the frame for receiveFrame() can
    # only come in once the link is up
    if not wait_link(p):
        record("cost", False, "no link 25 s after the start: the frame for receiveFrame() cannot come in")
        return
    sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sk.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sk.bind((host_ip, 0))
    rows, problems = [], []
    try:
        for seq, plen in enumerate(payloads):
            line = cost_run(p, sk, seq, plen)
            if line.startswith("COSTERR"):
                record("cost", False, "the board answered '%s'%s" % (
                    line, " (build with --flash --profile)" if "noprofile" in line else ""))
                return
            d = kv(line)
            rows.append(d)
            problems += ["%d B payload: %s" % (plen, x) for x in cost_problems(d)]
            info("cost", "%4d B in, %4d B out at %.1f MHz: available() %d commands, %d us; receiveFrame() %d, %d us; "
                 "upload %d, %d us; start %d, %d us; wait %d reads of EIR, %d us" %
                 (num(d, "rxlen"), num(d, "uplen"), num(d, "spihz") / 1e6, num(d, "avail"), num(d, "avail_us"),
                  num(d, "rx"), num(d, "rx_us"), num(d, "up"), num(d, "up_us"), num(d, "start"), num(d, "start_us"),
                  num(d, "eir"), num(d, "wait_us")))
    finally:
        sk.close()
    budgets = [cost_budget(num(d, "rxlen"), num(d, "uplen")) for d in rows]
    cheaper = sorted({step for d, b in zip(rows, budgets) for step in b if num(d, step) < b[step]})
    if cheaper:
        info("cost", "fewer commands than cost_budget() allows, in %s: lower the budget in the same change that made "
             "the code cheaper" % ", ".join(cheaper))

    def taken(step):
        return ", ".join("%d/%d" % (num(d, step), b[step]) for d, b in zip(rows, budgets))

    record("cost", not problems,
           "commands taken/allowed: available() %s; receiveFrame() of %s bytes %s; upload of %s bytes %s; start %s%s" %
           (taken("avail"), ", ".join(d["rxlen"] for d in rows), taken("rx"), ", ".join(d["uplen"] for d in rows),
            taken("up"), taken("start"), "; PROBLEMS: " + "; ".join(problems) if problems else ""))


def t_pioshare(p, ip):
    """Other users of the PIO fill all the other state machines; the one of the driver has to stay with it."""
    t = kv(p.cmd("PIOSHARE", "PIOSHARE", 20))
    replies = ping_replies(ip)
    record("pioshare", num(t, "started") >= 5 and num(t, "claimed") == 1 and num(t, "rev") == 6 and num(t, "after") == 6 and
           num(t, "io") == 0 and replies == 5,
           "%s other PIO users started; the state machine of the driver stayed claimed; revision read with them "
           "running 0x%02X, after 0x%02X; I/O errors %s; ping %d/5" %
           (t["started"], num(t, "rev"), num(t, "after"), t["io"], replies))


def t_stall(p, ip):
    """The state machine of the driver is stopped: the call that finds out must return, and the driver must come back."""
    before = stats(p)  # earlier tests may have recovered the chip already (setmac does)
    t = kv(p.cmd("STALL", "STALLDONE", 20))
    io, rec = num(t, "io") - num(before, "io"), num(t, "rec") - num(before, "rec")
    link = wait_link(p)
    replies = ping_replies(ip) if link else 0
    record("stall", num(t, "first_us") < 100000 and num(t, "first") == 0xFF and num(t, "second") == 6 and io == 1 and
           rec == 1 and link and replies == 5,
           "state machine stopped: the first call came back after %.1f ms with the failure value, the next one "
           "brought the chip back in %s ms (io errors +%d, recoveries +%d); link %s, ping %d/5" %
           (num(t, "first_us") / 1000, t["second_ms"], io, rec, "up" if link else "DOWN", replies))


def t_lifecycle(p, ip):
    """end() and begin() again: every end() gives the state machine back, and the chip works after the last begin()."""
    t = kv(p.cmd("LIFECYCLE", "LIFECYCLE", 30))
    link = wait_link(p)
    replies = ping_replies(ip) if link else 0
    record("lifecycle", num(t, "freed") == 1 and num(t, "began") == 1 and num(t, "rev") == 6 and num(t, "claimed") == 1 and
           link and replies == 5,
           "%s x end()+begin(): the state machine was given back each time (%s), began %s; link %s, ping %d/5" %
           (t["cycles"], "yes" if num(t, "freed") else "NO", "ok" if num(t, "began") else "FAILED",
            "up" if link else "DOWN", replies))


def t_txmix(p, host_ip, count=240):
    """With the longest receive buffer only 1536 bytes are left to send from: frames of every length, the longest
    included, still have to find room, over and over."""
    results = []
    for rx_size in (0x1A00, 0x1600):
        p.cmd("RESET", "OK")
        sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sk.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sk.bind(("", PORT_TX))
        p.send("TXMIX %d %d" % (rx_size, count))
        got, dup, corrupt = set(), 0, 0
        first = True
        while len(got) < count:
            sk.settimeout(15.0 if first else 1.5)  # the board starts over first, and the link takes a while
            try:
                data, _ = sk.recvfrom(2048)
            except socket.timeout:
                break
            first = False
            seq = check_payload(data)
            if seq is None:
                corrupt += 1
            elif seq in got:
                dup += 1
            else:
                got.add(seq)
        sk.close()
        done = kv(p.expect("TXMIXDONE", 60))
        wait_link(p)
        results.append((rx_size, len(got), dup, corrupt, num(done, "sent"), num(done, "fail"), num(done, "began"),
                        num(done, "restored")))
    good = all(r[1] == count and r[2] == 0 and r[3] == 0 and r[5] == 0 and r[6] == 1 and r[7] == 1 for r in results)
    record("txmix", good, "; ".join(
        "receive buffer 0x%04X (transmit area %d bytes): sendFrame ok=%d failed=%d, host received %d/%d, "
        "duplicates=%d corrupted=%d" % (r[0], 0x2000 - r[0], r[4], r[5], r[1], count, r[2], r[3]) for r in results))


def t_txring(p, host_ip, count=240):
    """Frames go out the way the FreeRTOS driver sends them: the next one is uploaded while the one before is on the
    wire. With a receive buffer of 0x1000, and of 0x141C, the default, where two full-size frames fill the transmit
    area to its last byte, the next one always finds room and no upload has to wait for the wire. With longer receive buffers, frames of mixed lengths have to find room
    after the pointers wrapped, and none may be dropped; with 0x145A, seven frames used to make the transmit area give
    the next frame bytes of the one on the wire, which the board finds from the ids of the frames. Every frame has to
    arrive intact."""
    results = []
    for rx_size, pattern in ((0x1000, 0), (0x141C, 0), (0x1600, 1), (0x1A00, 1), (0x145A, 2)):
        p.cmd("RESET", "OK")
        sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sk.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sk.bind(("", PORT_TX))
        p.send("TXRING %d %d %d" % (rx_size, count, pattern))
        got, dup, corrupt = set(), 0, 0
        first = True
        while len(got) < count:
            sk.settimeout(15.0 if first else 1.5)  # the board starts over first, and the link takes a while
            try:
                data, _ = sk.recvfrom(2048)
            except socket.timeout:
                break
            first = False
            seq = check_payload(data)
            if seq is None:
                corrupt += 1
            elif seq in got:
                dup += 1
            else:
                got.add(seq)
        sk.close()
        done = kv(p.expect("TXRINGDONE", 60))
        wait_link(p)
        results.append((rx_size, pattern, len(got), dup, corrupt, done))
    good = True
    for rx_size, pattern, got, dup, corrupt, done in results:
        good = good and got == count and dup == 0 and corrupt == 0 and num(done, "sent") == count and \
            num(done, "fail") == 0 and num(done, "dropped") == 0 and num(done, "overlaps") == 0 and \
            num(done, "began") == 1 and num(done, "restored") == 1
        if pattern == 0:
            good = good and num(done, "waited") == 0
    record("txring", good, "; ".join(
        "receive buffer 0x%04X (transmit area %d bytes), %s: sent %d, failed %d, dropped %d, uploads that waited for the "
        "wire %d, uploads over the frame on the wire %d; host received %d/%d, duplicates=%d corrupted=%d" %
        (rx_size, 0x2000 - rx_size, ("full-size frames", "mixed lengths", "the seven frames")[pattern],
         num(done, "sent"), num(done, "fail"), num(done, "dropped"), num(done, "waited"), num(done, "overlaps"), got,
         count, dup, corrupt)
        for rx_size, pattern, got, dup, corrupt, done in results))


def t_lostchip(p, ip):
    """The chip is reset behind the back of the driver, the way a glitch of its supply would: checkChip() has to notice,
    configure it again, and the board has to answer pings afterwards."""
    t = kv(p.cmd("LOSTCHIP", "LOSTCHIP", 20))
    link = wait_link(p)
    replies = ping_replies(ip) if link else 0
    record("lostchip", num(t, "alive_before") == 0 and num(t, "fixed") == 1 and num(t, "alive_after") == 1 and
           num(t, "resets") == 1 and num(t, "rev") == 6 and link and replies == 5,
           "chip reset behind the driver: it %s the reset, checkChip() took %s ms to configure it again (resets counted "
           "%s), revision 0x%02X; link %s, ping %d/5" %
           ("MISSED" if num(t, "alive_before") else "noticed", t["ms"], t["resets"], num(t, "rev"),
            "up" if link else "DOWN", replies))


def int_level(p, samples=5):
    """The level of the INT pin as the board sees it, sampled a few times over 0.2 s; 1 = idle."""
    levels = []
    for _ in range(samples):
        levels.append(num(stats(p), "int"))
        time.sleep(0.04)
    return min(levels), max(levels)


def t_irqidle(p, seconds=8):
    """With nothing sent to the board the interrupt must stay calm: every edge belongs to a frame or an
    event, and the line is high at the end."""
    p.cmd("RESET", "OK")
    s0 = stats(p)
    time.sleep(seconds)
    s1 = stats(p)
    irq = num(s1, "irq")  # counted since the RESET, like everything else in STAT
    frames, svc, events = num(s1, "frames"), num(s1, "svc"), num(s1, "lnk") + num(s1, "rxe")
    lo, hi = int_level(p)
    ok = irq <= frames + events + 2 and svc <= irq + 2 and lo == 1 and num(s1, "rxrst") == num(s0, "rxrst")
    record("irqidle", ok, "%d s quiet: %d edges on INT for %d ambient frames and %d events, %d service passes; "
           "line idle level %d..%d; receiver resets %d" %
           (seconds, irq, frames, events, svc, lo, hi, num(s1, "rxrst") - num(s0, "rxrst")))


def t_irqedge(p, host_ip, count=100, gap=0.05):
    """Frames far enough apart that each one has to raise the line by itself."""
    p.cmd("RESET", "OK")
    sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sk.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sk.bind((host_ip, 0))
    t0 = time.time()
    for seq in range(count):
        sk.sendto(make_payload(seq, 100), ("255.255.255.255", PORT_RX))
        time.sleep(gap)
    sk.close()
    time.sleep(0.5)
    s = stats(p)
    irq = num(s, "irq")
    lo, _ = int_level(p)
    ok = (num(s, "bench") == count and num(s, "gap") == 0 and num(s, "bad") == 0 and irq >= count and
          num(s, "svc") >= count * 0.95 and irq <= num(s, "frames") + num(s, "lnk") + num(s, "rxe") + 2 and lo == 1)
    record("irqedge", ok, "%d frames %d ms apart in %.1f s: received %s, %d edges, %s service passes, "
           "%s ambient frames; line idle level %d" %
           (count, gap * 1000, time.time() - t0, s["bench"], irq, s["svc"], num(s, "frames") - num(s, "bench"), lo))


def t_irqlink(p, ping_ip):
    """Power the PHY down and up: the interrupt has to report the link going away and coming back."""
    p.cmd("RESET", "OK")
    p.s.reset_input_buffer()
    p.rx = b""
    p.send("LINK")
    try:
        p.expect("LINKSTART", 5)
        end = p.expect("LINK", 30)  # LINKDONE or LINKFAIL
    except TimeoutError as e:
        record("irqlink", False, str(e))
        return
    d = kv(end)
    done = end.startswith("LINKDONE")
    time.sleep(1.0)
    r = subprocess.run(["ping", "-c", "5", "-i", "0.2", "-W", "1", ping_ip], capture_output=True, text=True)
    m = re.search(r"(\d+) packets transmitted, (\d+) received", r.stdout)
    got = int(m.group(2)) if m else 0
    link = num(kv(p.cmd("INFO", "INFO")), "link")
    ok = done and num(d, "down_ms") < 1000 and num(d, "lnk") >= 2 and link == 1 and got == 5
    record("irqlink", ok, "%s: link-down reported %s ms after the PHY was powered down, link-up %s ms after power-up, "
           "%s link interrupts; link=%d, ping after the cycle %d/5" %
           ("ok" if done else "no link-up interrupt", d.get("down_ms", "?"), d.get("up_ms", "?"), d.get("lnk", "?"),
            link, got))


def aopt(p, check, discard_every):
    """The options of the receive path of the board in its asynchronous mode (AOPT)."""
    reply = p.cmd("AOPT %d %d" % (check, discard_every), ("OK", "ASYNCERR"))
    if reply != "OK":
        raise RuntimeError("AOPT answered '%s'" % reply)


def async_enter(p):
    """ASYNC 1: the chip goes to the C driver in its asynchronous mode. The chip is reset, so wait for the link to come
    back. Returns (the board is in the mode, it can be used); a failure is recorded as asyncmode."""
    reply = p.cmd("ASYNC 1", ("OK", "ASYNCERR"), 30)
    if reply != "OK":
        record("asyncmode", False, "ASYNC 1 was answered with '%s'" % reply)
        return False, False
    if not wait_link(p):
        record("asyncmode", False, "no link 25 s after ASYNC 1")
        return True, False
    return True, True


def async_leave(p):
    """ASYNC 0: the Arduino class takes the chip back."""
    reply = p.cmd("ASYNC 0", ("OK", "ASYNCERR"), 30)
    if reply != "OK":
        record("asyncmode", False, "ASYNC 0 was answered with '%s'" % reply)
    wait_link(p)


def guarded(name, test):
    """A test that needs the board to answer: if the board stops, that is the result, and the next test still runs."""
    try:
        test()
    except (TimeoutError, RuntimeError, serial.SerialException) as e:
        record(name, False, "the test could not finish: %s" % e)


def t_asyncrx(p, host_ip, count=200):
    """The asynchronous receive path, with ERXRDPT read back from the chip after every frame: the chip needs it odd
    (silicon errata, issue 14), so the driver has to leave it odd."""
    aopt(p, 1, 0)
    p.cmd("RESET", "OK")
    s = stream_rx(p, host_ip, count, 0.01, SIZES)
    good = (num(s, "bench") == count and num(s, "dup") == 0 and num(s, "stale") == 0 and num(s, "gap") == 0 and
            num(s, "bad") == 0 and num(s, "hdrbad") == 0 and num(s, "waitto") == 0 and num(s, "pend") == 0 and
            num(s, "rdptchk") >= count and num(s, "rdpteven") == 0)
    record("asyncrx", good, "sent %d broadcast frames; board: ok=%s dup=%s stale=%s gap=%s bad=%s, bad headers=%s, "
           "wait timeouts=%s, frames left in the buffer=%s; ERXRDPT read back after %s frames, even %s times (the last "
           "0x%04X, after a frame whose next-packet pointer was 0x%04X)" %
           (count, s["bench"], s["dup"], s["stale"], s["gap"], s["bad"], s["hdrbad"], s["waitto"], s["pend"],
            s["rdptchk"], s["rdpteven"], num(s, "rdptlast"), num(s, "nextlast")))


def t_asyncdiscard(p, host_ip, count=100):
    """enc28j60_frame_discard() on every frame: none is delivered, ERXRDPT is odd after each, and the receiver is in
    step afterwards."""
    aopt(p, 1, 1)
    p.cmd("RESET", "OK")
    s = stream_rx(p, host_ip, count, 0.01, SIZES)
    aopt(p, 1, 0)
    p.cmd("RESET", "OK")
    s2 = stream_rx(p, host_ip, 50, 0.01, [100])
    good = (num(s, "bench") == 0 and num(s, "dis") >= count and num(s, "rdpteven") == 0 and num(s, "hdrbad") == 0 and
            num(s, "waitto") == 0 and num(s2, "bench") == 50 and num(s2, "gap") == 0 and num(s2, "bad") == 0 and
            num(s2, "dup") == 0 and num(s2, "stale") == 0 and num(s2, "hdrbad") == 0 and num(s2, "waitto") == 0)
    record("asyncdiscard", good, "sent %d broadcast frames to a board that drops every one unseen: delivered %s, "
           "discarded %s, bad headers=%s, wait timeouts=%s; ERXRDPT read back after %s frames, even %s times (the last "
           "0x%04X, after a frame whose next-packet pointer was 0x%04X); clean stream afterwards %s/50 (gap=%s)" %
           (count, s["bench"], s["dis"], s["hdrbad"], s["waitto"], s["rdptchk"], s["rdpteven"], num(s, "rdptlast"),
            num(s, "nextlast"), s2["bench"], s2["gap"]))


def t_asyncfast(p, host_ip, count=3000):
    """Small frames 1 ms apart, and nothing between one asynchronous call and the next: the driver is called again
    right after a frame was read, the way the FreeRTOS driver calls it."""
    aopt(p, 0, 0)
    p.cmd("RESET", "OK")
    stream_rx(p, host_ip, count, 0.001, [18, 100])
    wait_quiet(p)
    s = stats(p)
    good = (num(s, "bench") == count and num(s, "dup") == 0 and num(s, "stale") == 0 and num(s, "gap") == 0 and
            num(s, "bad") == 0 and num(s, "hdrbad") == 0 and num(s, "waitto") == 0 and num(s, "pend") == 0)
    record("asyncfast", good, "sent %d small broadcast frames 1 ms apart; board: ok=%s dup=%s stale=%s gap=%s bad=%s, "
           "bad headers=%s, wait timeouts=%s, still in the buffer=%s" %
           (count, s["bench"], s["dup"], s["stale"], s["gap"], s["bad"], s["hdrbad"], s["waitto"], s["pend"]))


def t_asyncecho(p, host_ip, pico_ip, count=200):
    """Unicast round trips with the reply sent right after the frame was read, so the next asynchronous call follows
    at once."""
    aopt(p, 0, 0)
    lost, bad, avg, s = run_echo(p, host_ip, pico_ip, count)
    record("asyncecho", lost == 0 and bad == 0 and num(s, "waitto") == 0 and num(s, "txto") == 0,
           "%d unicast round trips: lost=%d corrupted=%d avg rtt=%.1f ms; board: echo=%s txf=%s, wait timeouts=%s, "
           "upload slot timeouts=%s, bad headers=%s" %
           (count, lost, bad, avg, s["echo"], s["txf"], s["waitto"], s["txto"], s["hdrbad"]))


def t_asynctx(p, host_ip, count=200, plen=300, gap_us=3000):
    """Broadcast frames sent with enc28j60_frame_upload() and enc28j60_frame_tx(), one waiting and one on the wire,
    the transmit interrupt moving the next one to the transmitter."""
    aopt(p, 0, 0)
    got, dup, corrupt, sent, fail = run_txb(p, host_ip, count, plen, gap_us)
    s = stats(p)
    record("asynctx", got == count and dup == 0 and corrupt == 0 and fail == 0 and num(s, "waitto") == 0 and
           num(s, "txto") == 0,
           "board sent ok=%d failed=%d (%d B payload, %d us gap, broadcast); host received %d/%d, duplicates=%d "
           "corrupted=%d; transmit interrupts confirmed %s frames, wait timeouts=%s, upload slot timeouts=%s, "
           "uploads refused=%s" %
           (sent, fail, plen, gap_us, got, count, dup, corrupt, s["txconf"], s["waitto"], s["txto"], s["txupf"]))


def t_asyncflood(p, host_ip):
    """The flood of rxflood, through the asynchronous functions."""
    aopt(p, 0, 0)
    t_rxflood(p, host_ip, name="asyncflood", board_async=True)


def t_asynccycle(p, cycles=8):
    """The C driver started and stopped `cycles` times in its asynchronous mode: every start works, and at the end the
    same DMA channels are claimed and the same heap is free. A driver that keeps its channels runs out of them and the
    firmware panics: then the last ASYNCCYCLE_STEP line says how far it got."""
    seen = []
    p.s.reset_input_buffer()
    p.rx = b""
    p.send("ASYNCCYCLE %d" % cycles)
    try:
        line = p.expect(("ASYNCCYCLE n=", "ASYNCERR"), 30, seen)
    except (TimeoutError, serial.SerialException) as e:
        steps = [x.split()[-1] for x in seen if x.startswith("ASYNCCYCLE_STEP")]
        other = [x for x in seen if not x.startswith("ASYNCCYCLE_STEP")]
        record("asynccycle", False, "the board stopped answering (%s); the last step it reported was %s of %d%s" %
               (e, steps[-1] if steps else "none", cycles, "; it also printed: " + " | ".join(other[-3:]) if other else ""))
        return
    if line.startswith("ASYNCERR"):
        record("asynccycle", False, "the board answered '%s'" % line)
        return
    d = kv(line)
    good = (num(d, "n") == cycles and num(d, "ok") == cycles and num(d, "dma_after") == num(d, "dma_before") and
            num(d, "heap_after") == num(d, "heap_before") and num(d, "waitto") == 0 and num(d, "began") == 1)
    record("asynccycle", good, "%s of %s starts and stops worked; DMA channels claimed %s before and %s after, free heap "
           "%s B before and %s B after; wait timeouts=%s; the class began again: %s" %
           (d["ok"], d["n"], d["dma_before"], d["dma_after"], d["heap_before"], d["heap_after"], d["waitto"],
            "yes" if num(d, "began") else "NO"))


def selected_tests(tests, irq, profile):
    """The names of the tests to run: those in --tests, or all of them, with the groups that need an option only when it is
    given: the interrupt tests with --irq, and with --profile the tests that read the counters of the driver, which only a
    BenchTest compiled with -DENC28J60_PROFILE has (not the build that users get)."""
    if tests is None:
        return ALL_TESTS + (IRQ_TESTS if irq is not None else []) + (PROFILE_TESTS if profile else [])
    wanted = tests.split(",")
    for name in PROFILE_TESTS:
        if name in wanted and not profile:
            sys.exit("bench.py: --tests %s needs --profile: the test reads counters that the driver keeps only in a build "
                     "with -DENC28J60_PROFILE (--flash --profile makes it)" % name)
    return wanted


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", required=True, help="free IPv4 address on the LAN for the board")
    ap.add_argument("--port", help="serial port (default: first /dev/serial/by-id/*Pico*)")
    ap.add_argument("--fqbn", default="rp2040:rp2040:rpipico")
    ap.add_argument("--flash", action="store_true", help="compile and upload BenchTest first")
    ap.add_argument("--profile", action="store_true",
                    help="with --flash, build BenchTest with -DENC28J60_PROFILE; adds the cost test, which needs that build")
    ap.add_argument("--irq", type=int, metavar="GPIO", help="the board is the interrupt build, INT wired to this GPIO")
    ap.add_argument("--spi-hz", type=int, metavar="HZ", help="with --flash, build with this SPI clock instead of the default")
    ap.add_argument("--alt-ip", help="a second free IPv4 address, for the setmac test (default: --ip plus two)")
    ap.add_argument("--spi-assert-mhz", type=float, default=10.0,
                    help="the spi test fails if a clock up to this many MHz is not clean (default 10)")
    ap.add_argument("--tests", help="comma separated; default: all")
    args = ap.parse_args()
    wanted = selected_tests(args.tests, args.irq, args.profile)

    port = find_port(args.port)
    if args.flash:
        flash(port, args.fqbn, args.irq, spi_hz=args.spi_hz, profile=args.profile)
    host_ip = local_ip_for(args.ip)
    head, _, last = args.ip.rpartition(".")
    alt_ip = args.alt_ip or "%s.%d" % (head, int(last) + 2)
    print("board %s on %s, PC %s, serial %s\n" % (args.ip, "LAN", host_ip, port))

    p = Pico(port)
    p.cmd("ID", "READY", 5)
    link = t_init(p) if "init" in wanted else True
    if link:
        p.cmd("IP " + args.ip, "OK")
        if "cfgerr" in wanted:
            t_cfgerr(p)
        if "pioful" in wanted:
            t_pioful(p)
        if "spi" in wanted:
            t_spi(p, args.ip, args.spi_assert_mhz * 1e6, args.spi_hz or DEFAULT_SPI_HZ)
        if "ping" in wanted:
            t_ping(p, args.ip)
        if "rx" in wanted:
            t_rx(p, host_ip)
        if "echo" in wanted:
            t_echo(p, host_ip, args.ip)
        if "tx" in wanted:
            t_tx(p, host_ip, args.ip, "tx", 100, 300, 3000)
        if "txlong" in wanted:
            t_tx(p, host_ip, args.ip, "txlong", 400, 300, 3000)
        if "txburst" in wanted:
            t_tx(p, host_ip, args.ip, "txburst", 200, 100, 0)
        if "soak" in wanted:
            t_soak(p, host_ip)
        if "rxflood" in wanted:
            t_rxflood(p, host_ip)
        if "rxoverflow" in wanted:
            t_rxoverflow(p, host_ip, args.irq is not None)
        if "discard" in wanted:
            t_discard(p, host_ip, args.irq is not None)
        if "callback" in wanted:
            t_callback(p, host_ip, args.irq is not None)
        if "linkflag" in wanted:
            t_linkflag(p, args.ip)
        if "setmac" in wanted:
            t_setmac(p, args.ip, alt_ip)
        if "txb" in wanted:
            t_txb(p, host_ip, "txb", 100, 300, 3000)
        if "txblong" in wanted:
            t_txb(p, host_ip, "txblong", 400, 300, 3000)
        if "txbburst" in wanted:
            t_txb(p, host_ip, "txbburst", 200, 100, 0)
        if "txfull" in wanted:
            t_txfull(p, host_ip)
        if "cost" in wanted:
            guarded("cost", lambda: t_cost(p, host_ip))
        if "txstat" in wanted:
            t_txstat(p)
        if "pioshare" in wanted:
            t_pioshare(p, args.ip)
        if "stall" in wanted:
            t_stall(p, args.ip)
        if "lifecycle" in wanted:
            t_lifecycle(p, args.ip)
        if "txmix" in wanted:
            t_txmix(p, host_ip)
        if "txring" in wanted:
            t_txring(p, host_ip)
        if "lostchip" in wanted:
            t_lostchip(p, args.ip)
        if args.irq is not None:
            if "irqidle" in wanted:
                t_irqidle(p)
            if "irqedge" in wanted:
                t_irqedge(p, host_ip)
            if "irqlink" in wanted:
                t_irqlink(p, args.ip)
        # The asynchronous functions of the C driver: the six tests share one session in the board's asynchronous
        # mode. asynccycle starts and stops the driver by itself and goes last of all, because a driver that does not
        # give back its DMA channels can kill the firmware.
        async_wanted = [t for t in ASYNC_TESTS if t in wanted]
        if async_wanted:
            in_mode, usable = async_enter(p)
            if usable:
                runners = {
                    "asyncrx": lambda: t_asyncrx(p, host_ip),
                    "asyncdiscard": lambda: t_asyncdiscard(p, host_ip),
                    "asyncfast": lambda: t_asyncfast(p, host_ip),
                    "asyncecho": lambda: t_asyncecho(p, host_ip, args.ip),
                    "asynctx": lambda: t_asynctx(p, host_ip),
                    "asyncflood": lambda: t_asyncflood(p, host_ip),
                }
                for name in async_wanted:
                    guarded(name, runners[name])
            if in_mode:
                async_leave(p)
        if "asynccycle" in wanted:
            t_asynccycle(p)
    else:
        print("no link: skipping the traffic tests")

    print("\n%d/%d passed" % (sum(results), len(results)))
    sys.exit(0 if results and all(results) else 1)


if __name__ == "__main__":
    main()
