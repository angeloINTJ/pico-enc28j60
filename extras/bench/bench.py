#!/usr/bin/env python3
# Copyright 2026 Ângelo Moisés Alves
# Licensed under the MIT license, see the LICENSE file for details.
"""On-hardware self-test for PicoENC28J60.

Runs against a Pico flashed with extras/bench/BenchTest and wired to an
ENC28J60 that is cabled to the same LAN as this PC. It generates real traffic
(ping, UDP broadcast, UDP unicast) and checks what the board saw and sent.

    python3 extras/bench/bench.py --ip 192.168.3.77 --flash
    python3 extras/bench/bench.py --ip 192.168.3.77 --flash --irq 6    # INT wired to GP6

With --irq the board is built to be driven by the chip's INT pin: the main loop
talks to the chip only when the pin asks for it. All the tests above then run on
top of that, and three more (irqidle, irqedge, irqlink) check the interrupt itself.

Besides the traffic, the tests check what the driver does when things go wrong: a
begin() with no chip, with settings that make no sense and with no room in the PIO,
other users of the PIO, a state machine that stops, begin() after end(), and the
status of a transmitted frame.

Needs pyserial, arduino-cli (only with --flash) and a free IPv4 address on the
LAN for the board. No root required.
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

try:
    import serial
except ImportError:
    sys.exit("pyserial is required: pip install pyserial")

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SKETCH = os.path.join(REPO, "extras", "bench", "BenchTest")
PICO_MAC = "02:50:49:43:4f:01"
PORT_RX, PORT_ECHO, PORT_TX, PEER_PORT = 9999, 9997, 9998, 9998
SIZES = [10, 18, 100, 400, 800, 1200, 1472]  # UDP payload sizes
ALL_TESTS = ["init", "cfgerr", "pioful", "ping", "rx", "echo", "tx", "txlong", "txburst", "txb", "txblong",
             "txbburst", "soak", "rxflood", "txstat", "pioshare", "stall", "lifecycle", "txmix", "lostchip"]
IRQ_TESTS = ["irqidle", "irqedge", "irqlink"]  # only with --irq
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

    def expect(self, prefix, timeout=5.0):
        deadline = time.time() + timeout
        while True:
            line = self._readline(deadline)
            if line is None:
                raise TimeoutError("no '%s' reply within %ss" % (prefix, timeout))
            if line.startswith(prefix):
                return line

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
    return cands[0]


def flash(port, fqbn, irq_pin=None, sketch=SKETCH):
    """Compile a bench sketch against this checkout and upload it."""
    build = os.path.join(tempfile.gettempdir(), "picoenc-%s-build-%s" %
                         (os.path.basename(sketch), "irq%d" % irq_pin if irq_pin is not None else "poll"))
    cmd = ["arduino-cli", "compile", "--fqbn", fqbn, "--library", REPO, "--build-path", build]
    if irq_pin is not None:
        cmd += ["--build-property", "compiler.cpp.extra_flags=-DBENCH_IRQ_PIN=%d" % irq_pin]
    subprocess.run(cmd + [sketch], check=True)
    # The core's reset-and-deploy helper only works with the real device node,
    # not with the /dev/serial/by-id symlink.
    subprocess.run(["arduino-cli", "upload", "-p", os.path.realpath(port), "--fqbn", fqbn,
                    "--input-dir", build], check=True)
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
            num(s, "gap") == 0 and num(s, "bad") == 0)
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
            num(s, "gap") == 0 and num(s, "bad") == 0 and num(s, "rxz") == 0 and num(s, "runt") == 0)
    record("soak", good, "%d frames in %.0f s: ok=%s dup=%s stale=%s gap=%s bad=%s runt=%s recoveries=%s; "
           "ambient frames=%d" % (count, time.time() - t0, s["bench"], s["dup"], s["stale"], s["gap"], s["bad"],
                                  s["runt"], s["rxz"], num(s, "frames") - num(s, "bench")))


def t_rxflood(p, host_ip, count=3000):
    """Overload the receiver on purpose. Frames may be lost, but what arrives must be intact and the
    receiver must work normally afterwards."""
    p.cmd("RESET", "OK")
    # Big frames, no pacing: they reach the board back to back at the 10 Mbit/s of
    # its port, faster than the 3.5 MHz SPI link can drain the 4 KB receive buffer.
    s = stream_rx(p, host_ip, count, 0, [1472, 1200, 800], pace=False)
    intact = num(s, "dup") == 0 and num(s, "stale") == 0 and num(s, "bad") == 0
    got, lost = num(s, "bench"), count - num(s, "bench")
    ok1 = intact and got > 0 and lost > 0  # lost == 0 would mean the test did not overload anything
    wait_quiet(p)  # frames queued in the switch keep arriving for a while after the flood
    p.cmd("RESET", "OK")
    s2 = stream_rx(p, host_ip, 50, 0.01, [100])
    ok2 = (num(s2, "bench") == 50 and num(s2, "gap") == 0 and num(s2, "bad") == 0 and
           num(s2, "dup") == 0 and num(s2, "stale") == 0)
    record("rxflood", ok1 and ok2, "flood of %d frames: %d received intact, %d lost to overflow, "
           "dup=%s stale=%s bad=%s, recoveries=%s; clean stream afterwards: %s/50" %
           (count, got, lost, s["dup"], s["stale"], s["bad"], s["rxz"], s2["bench"]))


def t_echo(p, host_ip, pico_ip, count=200):
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


def run_txb(p, host_ip, count, plen, gap_us):
    """Have the board broadcast `count` frames; needs no RX path on the board."""
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
    done = kv(p.expect("TXDONE", 10))
    return len(got), dup, corrupt, num(done, "sent"), num(done, "fail")


def t_txb(p, host_ip, name, count, plen, gap_us):
    got, dup, corrupt, sent, fail = run_txb(p, host_ip, count, plen, gap_us)
    record(name, got == count and dup == 0 and corrupt == 0 and fail == 0,
           "board sendFrame ok=%d failed=%d (%d B payload, %d us gap, broadcast); host received %d/%d, "
           "duplicates=%d corrupted=%d" % (sent, fail, plen, gap_us, got, count, dup, corrupt))


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
    t = kv(p.cmd("STALL", "STALLDONE", 20))
    link = wait_link(p)
    replies = ping_replies(ip) if link else 0
    record("stall", num(t, "first_us") < 100000 and num(t, "first") == 0xFF and num(t, "second") == 6 and num(t, "io") == 1 and
           num(t, "rec") == 1 and link and replies == 5,
           "state machine stopped: the first call came back after %.1f ms with the failure value, the next one "
           "brought the chip back in %s ms (io errors %s, recoveries %s); link %s, ping %d/5" %
           (num(t, "first_us") / 1000, t["second_ms"], t["io"], t["rec"], "up" if link else "DOWN", replies))


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


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", required=True, help="free IPv4 address on the LAN for the board")
    ap.add_argument("--port", help="serial port (default: first /dev/serial/by-id/*Pico*)")
    ap.add_argument("--fqbn", default="rp2040:rp2040:rpipico")
    ap.add_argument("--flash", action="store_true", help="compile and upload BenchTest first")
    ap.add_argument("--irq", type=int, metavar="GPIO", help="the board is the interrupt build, INT wired to this GPIO")
    ap.add_argument("--tests", help="comma separated; default: all")
    args = ap.parse_args()
    if args.tests is None:
        args.tests = ",".join(ALL_TESTS + (IRQ_TESTS if args.irq is not None else []))

    port = find_port(args.port)
    if args.flash:
        flash(port, args.fqbn, args.irq)
    host_ip = local_ip_for(args.ip)
    print("board %s on %s, PC %s, serial %s\n" % (args.ip, "LAN", host_ip, port))

    p = Pico(port)
    p.cmd("ID", "READY", 5)
    wanted = args.tests.split(",")
    link = t_init(p) if "init" in wanted else True
    if link:
        p.cmd("IP " + args.ip, "OK")
        if "cfgerr" in wanted:
            t_cfgerr(p)
        if "pioful" in wanted:
            t_pioful(p)
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
        if "txb" in wanted:
            t_txb(p, host_ip, "txb", 100, 300, 3000)
        if "txblong" in wanted:
            t_txb(p, host_ip, "txblong", 400, 300, 3000)
        if "txbburst" in wanted:
            t_txb(p, host_ip, "txbburst", 200, 100, 0)
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
        if "lostchip" in wanted:
            t_lostchip(p, args.ip)
        if args.irq is not None:
            if "irqidle" in wanted:
                t_irqidle(p)
            if "irqedge" in wanted:
                t_irqedge(p, host_ip)
            if "irqlink" in wanted:
                t_irqlink(p, args.ip)
    else:
        print("no link: skipping the traffic tests")

    print("\n%d/%d passed" % (sum(results), len(results)))
    sys.exit(0 if results and all(results) else 1)


if __name__ == "__main__":
    main()
