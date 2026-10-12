#!/usr/bin/env python3
# Copyright 2026 Ângelo Moisés Alves
# Licensed under the MIT license, see the LICENSE file for details.
"""On-hardware self-test for PicoENC28J60lwIP, the lwIP network interface.

Runs against a Pico flashed with extras/bench/BenchLwip and wired to an ENC28J60
that is cabled to the same LAN as this PC. This side of the network pings the
board, opens TCP and UDP sockets to its echo servers, serves a TCP connection
that the board opens, and pulls the "cable" out (the PHY is powered down and up).

    python3 extras/bench/benchlwip.py --flash
    python3 extras/bench/benchlwip.py --flash --irq 6                  # INT wired to GP6
    python3 extras/bench/benchlwip.py --static-ip 192.168.3.78         # a free address, for the static test

Needs a DHCP server on the LAN, pyserial, arduino-cli (only with --flash), and the
`ping` command. No root required.
"""
import argparse
import ipaddress
import os
import random
import re
import socket
import subprocess
import sys
import threading
import time

import bench
from bench import Pico, REPO, find_port, flash, info, kv, local_ip_for, num, record

SKETCH = os.path.join(REPO, "extras", "bench", "BenchLwip")
PORT_TCP_ECHO, PORT_UDP_ECHO = 7007, 7008
ALL_TESTS = ["dhcp", "ping", "tcp", "tcppar", "tcpcycle", "udp", "pingout", "client", "udpsend", "chain", "dns",
             "flap", "lostchip", "static", "lifecycle", "heap"]


# --- patterns shared with BenchLwip.ino -------------------------------------
def down_byte(i):
    return (i * 31 + 7) & 0xFF


def up_byte(i):
    return (i * 17 + 3) & 0xFF


# --- helpers ------------------------------------------------------------------
class Ctx:
    """What the tests share: the serial link, the addresses, the options."""

    def __init__(self, p, host_ip, args):
        self.p = p
        self.host_ip = host_ip
        self.args = args
        self.board_ip = None
        self.gateway = None
        self.mask = None
        self.heap0 = None

    def info(self):
        return kv(self.p.cmd("INFO", "INFO", 10))

    def start_dhcp(self):
        t = kv(self.p.cmd("START dhcp", "START", 60))
        if t.get("ok") == "1":
            self.board_ip, self.gateway, self.mask = t["ip"], t["gw"], t["mask"]
        return t


def host_ping(ip, count=20, size=None, interval=0.2):
    cmd = ["ping", "-c", str(count), "-i", str(interval), "-W", "1"]
    if size:
        cmd += ["-s", str(size)]
    r = subprocess.run(cmd + [ip], capture_output=True, text=True)
    m = re.search(r"(\d+) packets transmitted, (\d+) received", r.stdout)
    return (int(m.group(1)), int(m.group(2))) if m else (count, 0)


def random_bytes(seed, n):
    rnd = random.Random(seed)
    if hasattr(rnd, "randbytes"):
        return rnd.randbytes(n)
    return bytes(rnd.getrandbits(8) for _ in range(n))


def tcp_echo(ip, total, seed, port=PORT_TCP_ECHO, timeout=60):
    """Send `total` bytes in chunks of random sizes to an echo server and read them back.
    Returns (ok, seconds, bytes read back, error)."""
    data = random_bytes(seed, total)
    rnd = random.Random(seed + 1)
    errors = []
    t0 = time.time()
    try:
        s = socket.create_connection((ip, port), timeout=10)
    except OSError as e:
        return False, 0.0, 0, "connect: %s" % e
    s.settimeout(timeout)

    def sender():
        try:
            pos = 0
            while pos < total:
                n = min(rnd.randint(1, 4096), total - pos)
                s.sendall(data[pos:pos + n])
                pos += n
        except OSError as e:
            errors.append("send: %s" % e)

    th = threading.Thread(target=sender)
    th.start()
    got = bytearray()
    try:
        while len(got) < total:
            b = s.recv(65536)
            if not b:
                break
            got += b
    except OSError as e:
        errors.append("recv: %s" % e)
    th.join(timeout)
    s.close()
    ok = bytes(got) == data and not errors
    return ok, time.time() - t0, len(got), "; ".join(errors)


# --- tests --------------------------------------------------------------------
def t_dhcp(c):
    t = c.start_dhcp()
    ok = t.get("ok") == "1"
    same = False
    if ok:
        c.host_ip = local_ip_for(t["ip"])
        net = ipaddress.ip_network("%s/%s" % (t["ip"], t["mask"]), strict=False)
        same = ipaddress.ip_address(c.host_ip) in net
        c.heap0 = num(c.info(), "heap")
    record("dhcp", ok and same,
           ("address %s from DHCP in %s ms (gateway %s, mask %s, DNS %s), on the network of the PC: %s" %
            (t["ip"], t["ms"], t["gw"], t["mask"], t["dns"], "yes" if same else "NO")) if ok else
           "no address: %s" % t.get("err", "?"))
    return ok and same


def t_ping(c):
    sent, got = host_ping(c.board_ip, 20)
    sent2, got2 = host_ping(c.board_ip, 5, size=1400)
    record("ping", got == sent and got2 == sent2,
           "%d/%d replies; %d/%d with 1400-byte payloads" % (got, sent, got2, sent2))
    sent3, got3 = host_ping(c.board_ip, 3, size=3000)
    info("pingfrag", "%d/%d replies to 3000-byte pings, which are fragmented on the wire" % (got3, sent3))


def t_tcp(c):
    b0 = c.info()
    ok, secs, got, err = tcp_echo(c.board_ip, 200000, 1)
    b1 = c.info()
    record("tcp", ok and num(b1, "tcp_accepted") - num(b0, "tcp_accepted") == 1,
           "200000 bytes of random data in chunks of 1..4096 bytes echoed through one connection in %.1f s "
           "(%.0f kB/s); %d read back%s" % (secs, 200 / max(secs, 0.001), got, "; " + err if err else ""))


def pingpong(ip, rounds, size, seed, port=PORT_TCP_ECHO, timeout=20):
    """One request at a time over one connection: send `size` random bytes, read them back, `rounds` times.
    Returns (rounds that came back intact, error)."""
    rnd = random.Random(seed)
    good = 0
    try:
        s = socket.create_connection((ip, port), timeout=10)
        s.settimeout(timeout)
        for _ in range(rounds):
            payload = bytes(rnd.getrandbits(8) for _ in range(size))
            s.sendall(payload)
            got = b""
            while len(got) < size:
                chunk = s.recv(size - len(got))
                if not chunk:
                    return good, "closed by the board"
                got += chunk
            if got != payload:
                return good, "round %d: the echo differs" % good
            good += 1
        s.close()
    except OSError as e:
        return good, str(e)
    return good, ""


def t_tcppar(c):
    """Several connections at once, each asking and getting an answer in turn, the way clients of a server do. (A
    bulk transfer through several connections at once is more than the chip can take: it can read a frame
    from its buffer in 3.5 ms, a frame arrives every 1.2 ms, and TCP backs off when it loses frames.)"""
    results = {}

    def run(i):
        results[i] = pingpong(c.board_ip, 150, 300, 20 + i)

    threads = [threading.Thread(target=run, args=(i,)) for i in range(3)]
    t0 = time.time()
    for th in threads:
        th.start()
    for th in threads:
        th.join(120)
    secs = time.time() - t0
    ok = len(results) == 3 and all(r[0] == 150 and not r[1] for r in results.values())
    record("tcppar", ok, "3 connections at once, 150 round trips of 300 random bytes each, in %.1f s: %s" %
           (secs, ", ".join("%d intact%s" % (r[0], " (%s)" % r[1] if r[1] else "") for _, r in sorted(results.items()))))


def t_tcpcycle(c):
    b0 = c.info()
    good = 0
    first_err = ""
    for i in range(40):
        try:
            s = socket.create_connection((c.board_ip, PORT_TCP_ECHO), timeout=5)
            s.settimeout(5)
            payload = bytes((i + j) & 0xFF for j in range(100))
            s.sendall(payload)
            got = b""
            while len(got) < len(payload):
                chunk = s.recv(200)
                if not chunk:
                    break
                got += chunk
            s.close()
            if got == payload:
                good += 1
            elif not first_err:
                first_err = "connection %d: echo differs" % i
        except OSError as e:
            if not first_err:
                first_err = "connection %d: %s" % (i, e)
    time.sleep(1.5)
    b1 = c.info()
    accepted = num(b1, "tcp_accepted") - num(b0, "tcp_accepted")
    drift = num(b1, "heap") - num(b0, "heap")
    record("tcpcycle", good == 40 and accepted == 40 and num(b1, "tcp_rejected") == num(b0, "tcp_rejected") and drift > -4096,
           "40 connections opened, used and closed: %d worked, the board accepted %d; free heap changed by %+d bytes%s" %
           (good, accepted, drift, "; " + first_err if first_err else ""))


def t_udp(c):
    sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sk.bind((c.host_ip, 0))
    sk.settimeout(1.0)
    lost = bad = total = 0
    for size in (1, 18, 100, 512, 1024, 1472):
        for seq in range(20):
            payload = bytes((seq * 7 + i) & 0xFF for i in range(size))
            sk.sendto(payload, (c.board_ip, PORT_UDP_ECHO))
            total += 1
            try:
                data, _ = sk.recvfrom(2048)
            except socket.timeout:
                lost += 1
                continue
            if data != payload:
                bad += 1
    record("udp", lost == 0 and bad == 0,
           "%d datagrams of 1 to 1472 bytes echoed: %d lost, %d corrupted" % (total, lost, bad))
    # larger than the MTU: the datagram is fragmented on the way out and back
    payload = random_bytes(99, 3000)
    sk.sendto(payload, (c.board_ip, PORT_UDP_ECHO))
    try:
        data, _ = sk.recvfrom(4096)
        info("udpfrag", "3000-byte datagram echoed %s" %
             ("intact" if data == payload else "with %d bytes, DIFFERENT" % len(data)))
    except socket.timeout:
        info("udpfrag", "3000-byte datagram (fragmented) was not echoed")
    sk.close()


def t_pingout(c):
    t = kv(c.p.cmd("PING " + c.host_ip, "PING", 30))
    record("pingout", num(t, "ok") == 5, "the board pinged this PC: %s of 5 replies" % t["ok"])


def t_client(c):
    down = up = 100000
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((c.host_ip, 0))
    srv.listen(1)
    srv.settimeout(30)
    port = srv.getsockname()[1]
    res = {}

    def serve():
        try:
            conn, _ = srv.accept()
        except OSError as e:
            res["err"] = "accept: %s" % e
            return
        conn.settimeout(30)
        try:
            conn.sendall(bytes(down_byte(i) for i in range(down)))
            got = bytearray()
            while len(got) < up:
                b = conn.recv(65536)
                if not b:
                    break
                got += b
            res["got"] = len(got)
            res["bad"] = sum(1 for i, x in enumerate(got) if x != up_byte(i))
        except OSError as e:
            res["err"] = str(e)
        finally:
            conn.close()

    th = threading.Thread(target=serve)
    th.start()
    t = kv(c.p.cmd("CLIENT %s %d %d %d" % (c.host_ip, port, down, up), "CLIENT", 90))
    th.join(40)
    srv.close()
    ok = t.get("ok") == "1" and res.get("got") == up and res.get("bad") == 0
    record("client", ok,
           "the board connected to this PC (%s ms), read %s bytes (%s wrong) in %s ms and sent %s bytes in %s ms; "
           "this side read %s bytes (%s wrong)%s" %
           (t.get("connect_ms", "?"), t.get("down", "?"), t.get("bad", "?"), t.get("down_ms", "?"), t.get("up", "?"),
            t.get("up_ms", "?"), res.get("got", "?"), res.get("bad", "?"), "; " + res["err"] if "err" in res else ""))


def t_udpsend(c):
    count, size = 100, 200
    sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sk.bind((c.host_ip, 0))
    sk.settimeout(0.3)
    port = sk.getsockname()[1]
    seen, bad = set(), 0
    stop = threading.Event()

    def receive():
        nonlocal bad
        while not stop.is_set():
            try:
                data, _ = sk.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                return
            if len(data) != size or data[:4] != b"UDPB":
                bad += 1
                continue
            i = int.from_bytes(data[4:8], "big")
            if any(data[j] != (i * 13 + j) & 0xFF for j in range(8, size)):
                bad += 1
            else:
                seen.add(i)

    th = threading.Thread(target=receive)
    th.start()
    t = kv(c.p.cmd("UDPSEND %s %d %d %d 5" % (c.host_ip, port, count, size), "UDPSEND", 30))
    time.sleep(0.7)
    stop.set()
    th.join(2)
    sk.close()
    record("udpsend", t.get("ok") == "1" and len(seen) == count and bad == 0,
           "the board sent %s datagrams of %d bytes: %d arrived, %d corrupted" % (t.get("sent", "?"), size, len(seen), bad))


def t_chain(c):
    count = 20
    sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sk.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sk.bind(("", 0))
    sk.settimeout(0.5)
    port = sk.getsockname()[1]
    seen, bad = set(), 0
    stop = threading.Event()

    def receive():
        nonlocal bad
        while not stop.is_set():
            try:
                data, _ = sk.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                return
            if len(data) != 300 or data[:4] != b"CHAN":
                bad += 1
                continue
            i = int.from_bytes(data[4:8], "big")
            if any(data[j] != (i * 7 + j) & 0xFF for j in range(8, 300)):
                bad += 1
            else:
                seen.add(i)

    th = threading.Thread(target=receive)
    th.start()
    t = kv(c.p.cmd("CHAIN %d %d" % (port, count), "CHAIN", 30))
    time.sleep(0.7)
    stop.set()
    th.join(2)
    sk.close()
    record("chain", t.get("ok") == "1" and len(seen) == count and bad == 0,
           "%s datagrams sent as chains of three buffers: %d arrived whole, %d corrupted" %
           (t.get("sent", "?"), len(seen), bad))


def t_lostchip(c):
    t = kv(c.p.cmd("LOSTCHIP", "LOSTCHIP", 40))
    sent = got = 0
    echo_ok = False
    link_ok = False
    deadline = time.time() + 30
    while time.time() < deadline:  # the link goes down with the reset and takes a while to come back
        sent, got = host_ping(c.board_ip, 3)
        if got == sent:
            link_ok = True
            break
    if link_ok:
        echo_ok = tcp_echo(c.board_ip, 20000, 9)[0]
    record("lostchip", num(t, "found") == 1 and num(t, "alive") == 1 and num(t, "resets") == 1 and link_ok and echo_ok,
           "the chip was reset behind the back of the interface: it noticed after %s ms and configured it again "
           "(resets counted %s); then ping %s, TCP echo %s" %
           (t["found_ms"], t["resets"], "ok" if link_ok else "FAILED", "ok" if echo_ok else "FAILED"))


def t_dns(c):
    name = c.args.dns_name or "example.com"
    t = kv(c.p.cmd("DNS " + name, "DNS", 30))
    detail = "%s -> %s in %s ms" % (name, t.get("ip", "?"), t.get("ms", "?"))
    if c.args.dns_name:
        record("dns", t.get("ok") == "1", detail)
    else:
        info("dns", detail + (" (needs a DNS server that answers; pass --dns-name to make it a test)"
                              if t.get("ok") != "1" else ""))


def t_flap(c):
    t = kv(c.p.cmd("FLAP", "FLAP", 90))
    if t.get("ip"):
        c.board_ip = t["ip"]
    sent, got = host_ping(c.board_ip, 5)
    echo_ok = tcp_echo(c.board_ip, 20000, 3)[0]
    dhcp = num(t, "dhcp") == 1
    ok = (num(t, "down") == 1 and num(t, "up") == 1 and num(t, "cb_down") >= 1 and num(t, "cb_up") >= 1 and
          (not dhcp or (num(t, "rebound") == 1 and t["xid0"] != t["xid1"])) and got == sent and echo_ok)
    if not dhcp:
        renewal = "not in use"
    elif num(t, "rebound") == 1 and t["xid0"] != t["xid1"]:
        renewal = "asked again (transaction %s -> %s, bound after %s ms)" % (t["xid0"], t["xid1"], t["rebind_ms"])
    else:
        renewal = "NOT renewed (transaction %s)" % t["xid1"]
    record("flap", ok,
           "link down seen after %s ms, up after %s ms; lwIP was told %s down, %s up; DHCP %s; then ping %d/%d, TCP echo %s" %
           (t["down_ms"], t["up_ms"], t["cb_down"], t["cb_up"], renewal, got, sent, "ok" if echo_ok else "FAILED"))


def t_static(c):
    if not c.args.static_ip:
        info("static", "skipped: pass --static-ip with a free address of the LAN")
        return
    c.p.cmd("STOP", "STOP", 30)
    t = kv(c.p.cmd("START static %s %s %s" % (c.args.static_ip, c.gateway, c.mask), "START", 60))
    ok = t.get("ok") == "1" and t.get("ip") == c.args.static_ip
    sent = got = 0
    echo_ok = False
    if ok:
        sent, got = host_ping(c.args.static_ip, 10)
        echo_ok = tcp_echo(c.args.static_ip, 20000, 5)[0]
    # back to DHCP for the tests that follow
    c.p.cmd("STOP", "STOP", 30)
    back = c.start_dhcp()
    record("static", ok and got == sent and echo_ok and back.get("ok") == "1",
           "static address %s: configured %s; ping %d/%d, TCP echo %s; DHCP again afterwards: %s" %
           (c.args.static_ip, "ok" if ok else "FAILED (%s)" % t.get("err", t.get("ip", "?")), got, sent,
            "ok" if echo_ok else "FAILED", "ok" if back.get("ok") == "1" else "FAILED"))


def t_lifecycle(c):
    heaps = []
    ok = True
    for _ in range(3):
        stop = kv(c.p.cmd("STOP", "STOP", 30))
        heaps.append(num(stop, "heap_after"))
        t = c.start_dhcp()
        ok = ok and t.get("ok") == "1"
    sent, got = host_ping(c.board_ip, 5) if ok else (5, 0)
    echo_ok = tcp_echo(c.board_ip, 20000, 7)[0] if ok else False
    drift = heaps[-1] - heaps[0]
    record("lifecycle", ok and got == sent and echo_ok and drift > -4096,
           "3 x end()+begin(): every one got an address; ping %d/%d, TCP echo %s; free heap after end() changed by %+d "
           "bytes between the first and the last" % (got, sent, "ok" if echo_ok else "FAILED", drift))


def t_heap(c):
    now = num(c.info(), "heap")
    drift = now - c.heap0
    record("heap", drift > -16384, "free heap %d bytes, %+d since the first address" % (now, drift))


TESTS = {
    "dhcp": t_dhcp, "ping": t_ping, "tcp": t_tcp, "tcppar": t_tcppar, "tcpcycle": t_tcpcycle, "udp": t_udp,
    "pingout": t_pingout, "client": t_client, "udpsend": t_udpsend, "chain": t_chain, "dns": t_dns, "flap": t_flap,
    "lostchip": t_lostchip,
    "static": t_static, "lifecycle": t_lifecycle, "heap": t_heap,
}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port (default: first /dev/serial/by-id/*Pico*)")
    ap.add_argument("--fqbn", default="rp2040:rp2040:rpipico")
    ap.add_argument("--flash", action="store_true", help="compile and upload BenchLwip first")
    ap.add_argument("--irq", type=int, metavar="GPIO", help="the board is the interrupt build, INT wired to this GPIO")
    ap.add_argument("--spi-hz", type=int, metavar="HZ", help="with --flash, build with this SPI clock instead of the default")
    ap.add_argument("--tests", help="comma separated; default: all")
    ap.add_argument("--static-ip", help="a free address of the LAN, for the static test")
    ap.add_argument("--dns-name", help="a name to resolve; makes the dns test count")
    args = ap.parse_args()
    wanted = (args.tests or ",".join(ALL_TESTS)).split(",")
    for name in wanted:
        if name not in TESTS:
            sys.exit("unknown test '%s'; the tests are: %s" % (name, ", ".join(ALL_TESTS)))

    port = find_port(args.port)
    if args.flash:
        flash(port, args.fqbn, args.irq, SKETCH, args.spi_hz)
    p = Pico(port)
    p.cmd("ID", "READY", 10)

    c = Ctx(p, None, args)
    if "dhcp" not in wanted:
        wanted.insert(0, "dhcp")  # everything else needs an address
    for name in wanted:
        if name != "dhcp" and c.board_ip is None:
            break  # no address, nothing to test
        TESTS[name](c)
        if name == "dhcp" and c.board_ip is not None:
            print("board %s, PC %s, serial %s\n" % (c.board_ip, c.host_ip, port))
    final = bench.results
    print("\n%d/%d passed" % (sum(final), len(final)))
    sys.exit(0 if final and all(final) else 1)


if __name__ == "__main__":
    main()
