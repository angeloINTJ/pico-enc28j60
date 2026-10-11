#!/usr/bin/env python3
"""HTTP test for examples/WebServerLedToggle running on a real board.

    python3 extras/bench/webtest.py --ip 192.168.3.39              # about a minute
    python3 extras/bench/webtest.py --ip 192.168.3.39 --soak 300   # plus a 5 minute soak

The board prints its address on the serial port once DHCP has given it one.
Checks, using nothing but Python 3:

  * 150 sequential GETs of each page: no failure and no multi-second stall
  * 1 to 4 clients in parallel, the way a browser opens connections
  * 100 x POST /toggle leave the LED and the level read back from the pin consistent
  * with --soak: a long run of mixed requests during which the free heap must not shrink
"""
import argparse
import collections
import concurrent.futures as cf
import json
import sys
import time
import urllib.error
import urllib.request


class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, *args, **kwargs):
        return None


OPENER = urllib.request.build_opener(NoRedirect)
results = []


def fetch(host, path, method="GET", timeout=6.0):
    """Returns (status, body, seconds, error)."""
    t0 = time.time()
    try:
        req = urllib.request.Request("http://%s%s" % (host, path), method=method,
                                     data=b"" if method == "POST" else None)
        with OPENER.open(req, timeout=timeout) as r:
            return r.status, r.read(), time.time() - t0, None
    except urllib.error.HTTPError as e:  # a 303 lands here: redirects are not followed
        return e.code, b"", time.time() - t0, None
    except Exception as e:
        return 0, b"", time.time() - t0, "%s: %s" % (type(e).__name__, str(e)[:60])


def status(host):
    code, body, _, err = fetch(host, "/status")
    if err or code != 200:
        raise RuntimeError("GET /status failed: %s %s" % (code, err))
    return json.loads(body)


def percentile(sorted_values, q):
    return sorted_values[min(len(sorted_values) - 1, int(q * len(sorted_values)))] * 1000


def record(name, ok, detail):
    results.append(ok)
    print("[%s] %-28s %s" % ("PASS" if ok else "FAIL", name, detail))


def summarize(name, rows, limit_s=1.0):
    ok = [r for r in rows if r[3] is None]
    failures = collections.Counter(r[3] for r in rows if r[3] is not None)
    lat = sorted(r[2] for r in ok)
    stalls = sum(1 for r in ok if r[2] > limit_s)
    detail = "n=%d ok=%d p50=%.0f ms p95=%.0f ms max=%.0f ms" % (
        len(rows), len(ok), percentile(lat, .5) if lat else 0, percentile(lat, .95) if lat else 0,
        lat[-1] * 1000 if lat else 0)
    if failures:
        detail += " failures=%s" % dict(failures)
    if stalls:
        detail += " (%d slower than %.0f s)" % (stalls, limit_s)
    record(name, len(ok) == len(rows) and stalls == 0, detail)


def t_sequential(host, path, n=150):
    summarize("GET %s x%d" % (path, n), [fetch(host, path) for _ in range(n)])


def t_parallel(host, workers, rounds=6, per_round=40):
    rows = []
    for _ in range(rounds):
        with cf.ThreadPoolExecutor(workers) as ex:
            rows += list(ex.map(lambda i: fetch(host, "/" if i % 2 else "/status"), range(per_round)))
    summarize("%d client(s) in parallel" % workers, rows)


def t_toggle(host, n=100):
    before = status(host)
    rows = [fetch(host, "/toggle", "POST") for _ in range(n)]
    after = status(host)
    redirected = sum(1 for r in rows if r[0] == 303)
    expected = bool(before["led"]) ^ (redirected % 2 == 1)
    consistent = after["led"] == expected and after["gpio"] == int(expected)
    record("POST /toggle x%d" % n, redirected == n and consistent,
           "%d redirects; LED %s -> %s (expected %s); pin level read back = %s" %
           (redirected, before["led"], after["led"], expected, after["gpio"]))


def t_soak(host, seconds):
    heaps, fails, requests = [], 0, 0
    deadline = time.time() + seconds
    while time.time() < deadline:
        s = status(host)
        heaps.append(s["free_heap"])
        for path, method in (("/", "GET"), ("/toggle", "POST"), ("/favicon.ico", "GET")):
            code, _, dt, err = fetch(host, path, method)
            requests += 1
            if err or dt > 2.0:
                fails += 1
        time.sleep(0.5)
    tenth = max(1, len(heaps) // 10)
    first, last = sum(heaps[:tenth]) / tenth, sum(heaps[-tenth:]) / tenth
    leak = first - last
    record("soak %ds" % seconds, fails == 0 and leak < 2048,
           "%d requests, %d failed or slower than 2 s; free heap %d -> %d bytes (min %d)" %
           (requests, fails, first, last, min(heaps)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", required=True, help="address of the board")
    ap.add_argument("--soak", type=int, default=0, metavar="SECONDS", help="also run a soak of this length")
    args = ap.parse_args()

    s = status(args.ip)
    print("board %s, up for %.0f s, free heap %d bytes\n" % (args.ip, s["uptime_ms"] / 1000, s["free_heap"]))
    t_sequential(args.ip, "/")
    t_sequential(args.ip, "/status")
    for workers in (1, 2, 3, 4):
        t_parallel(args.ip, workers)
    t_toggle(args.ip)
    if args.soak:
        t_soak(args.ip, args.soak)
    print("\n%d/%d passed" % (sum(results), len(results)))
    sys.exit(0 if all(results) else 1)


if __name__ == "__main__":
    main()
