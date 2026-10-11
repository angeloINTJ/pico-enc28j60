#!/usr/bin/env python3
"""On-hardware self-test for PicoENC28J60.

Runs against a Pico flashed with extras/bench/BenchTest and wired to an
ENC28J60 that is cabled to the same LAN as this PC. It generates real traffic
(ping, UDP broadcast, UDP unicast) and checks what the board saw and sent.

    python3 extras/bench/bench.py --ip 192.168.3.77 --flash

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
ALL_TESTS = ["init", "ping", "rx", "echo", "tx", "txlong", "txburst", "txb", "txblong", "txbburst",
             "soak", "rxflood"]
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


def flash(port, fqbn):
    build = os.path.join(tempfile.gettempdir(), "picoenc-bench-build")
    subprocess.run(["arduino-cli", "compile", "--fqbn", fqbn, "--library", REPO,
                    "--build-path", build, SKETCH], check=True)
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
           "initialised=%s revision=0x%02X mac=%s" % (d["init"], rev, d["mac"]))
    record("errpath", num(d, "probe_ok") == 0 and num(d, "probe_ms") < 500 and num(d, "init") == 1,
           "begin() with the CS pin on an unconnected GPIO %s after %s ms; the real begin() right after worked"
           % ("failed" if num(d, "probe_ok") == 0 else "SUCCEEDED (wrong)", d["probe_ms"]))
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


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", required=True, help="free IPv4 address on the LAN for the board")
    ap.add_argument("--port", help="serial port (default: first /dev/serial/by-id/*Pico*)")
    ap.add_argument("--fqbn", default="rp2040:rp2040:rpipico")
    ap.add_argument("--flash", action="store_true", help="compile and upload BenchTest first")
    ap.add_argument("--tests", default=",".join(ALL_TESTS), help="comma separated; default: all")
    args = ap.parse_args()

    port = find_port(args.port)
    if args.flash:
        flash(port, args.fqbn)
    host_ip = local_ip_for(args.ip)
    print("board %s on %s, PC %s, serial %s\n" % (args.ip, "LAN", host_ip, port))

    p = Pico(port)
    p.cmd("ID", "READY", 5)
    wanted = args.tests.split(",")
    link = t_init(p) if "init" in wanted else True
    if link:
        p.cmd("IP " + args.ip, "OK")
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
    else:
        print("no link: skipping the traffic tests")

    print("\n%d/%d passed" % (sum(results), len(results)))
    sys.exit(0 if results and all(results) else 1)


if __name__ == "__main__":
    main()
