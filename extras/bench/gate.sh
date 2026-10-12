#!/bin/bash
# Copyright 2026 Ângelo Moisés Alves
# Licensed under the MIT license, see the LICENSE file for details.
#
# The hardware gate: every suite of extras/bench on a board, one after the other, each into its own
# log, and a summary of which passed. A change to the driver, to the PIO program or to the lwIP
# interface goes in only after it passed here. See --help and "The whole gate" in
# extras/bench/README.md.
#
# There is no `set -e` on purpose: a suite that fails is a result to write down, and the next one
# still runs.
set -uo pipefail

here=$(dirname "$(readlink -f "$0")")
repo=$(cd "$here/../.." && pwd)
default_out=${TMPDIR:-/tmp}/picoenc-gate

usage() {
    cat <<EOF
Usage: gate.sh --ip ADDR [options]

Run the suites of extras/bench on a Pico with an ENC28J60, one after the other, each into its own
log, and write a summary of which passed. The suites compile and flash the checkout this script is
in. The exit status is 0 only if every suite that ran passed.

Suites, in this order: bench_poll, bench_cost and bench_irq (bench.py), lwip_poll and lwip_irq
(benchlwip.py), web_poll and web_irq (webtest.py on examples/WebServerLedToggle). bench_cost runs
only the cost test, in the build with the counters of the driver (bench.py --flash --profile),
which is not the build that users get. The _irq suites run with --irq, the web suites with
--web-mac; the summary names the ones that did not run, and why.

Options:
  --ip ADDR        a free IPv4 address for the board in bench.py (required). ADDR plus 2 has to be
                   free too: the setmac test uses it
  --lwip-ip ADDR   a free address for the static address test of benchlwip.py (default: ADDR plus 1)
  --irq GPIO       the INT pin of the chip is wired to this GPIO: every suite also runs in its
                   interrupt build
  --web-mac MAC    the MAC address of the web example. Runs the web suites, and finds the board by
                   it after a ping sweep of the /24 of ADDR (how to get it: extras/bench/README.md)
  --soak SECONDS   the length of the soak in each web suite (default: 240)
  --spi-hz HZ      run bench.py and benchlwip.py at this SPI clock. The web example has no such
                   option, so the web suites do not run
  --port PORT      the serial port of the Pico (default: the only /dev/serial/by-id/*Pico*; with
                   more than one Pico connected the gate asks for it)
  --out DIR        where the logs and summary.txt go (default: $default_out)
  --fqbn FQBN      the board (default: rp2040:rp2040:rpipico)
  -h, --help       this text

Needs Linux, arduino-cli with the rp2040 core, pyserial and ping; with --web-mac also curl and ip.
About 16 minutes with --irq and --web-mac. summary.txt ends with DONE if the gate got to the end.
EOF
}

die() {
    echo "gate.sh: $*" >&2
    echo "Try 'gate.sh --help'." >&2
    exit 2
}

valid_ipv4() { # valid_ipv4 ADDR: four numbers from 0 to 255
    local number
    [[ $1 =~ ^([0-9]{1,3})\.([0-9]{1,3})\.([0-9]{1,3})\.([0-9]{1,3})$ ]] || return 1
    for number in "${BASH_REMATCH[@]:1}"; do
        ((10#$number <= 255)) || return 1
    done
}

ip_plus() { # ip_plus ADDR N: ADDR with N added to the last number, if that is still a host address
    local last=$((10#${1##*.} + $2))
    ((last < 255)) || return 1
    echo "${1%.*}.$last"
}

ip= lwip_ip= irq= web_mac= soak=240 spi_hz= port= out=$default_out fqbn=rp2040:rp2040:rpipico
while [[ $# -gt 0 ]]; do
    case $1 in
        -h | --help)
            usage
            exit 0
            ;;
        --ip | --lwip-ip | --irq | --web-mac | --soak | --spi-hz | --port | --out | --fqbn)
            [[ $# -ge 2 ]] || die "$1 needs a value"
            case $1 in
                --ip) ip=$2 ;;
                --lwip-ip) lwip_ip=$2 ;;
                --irq) irq=$2 ;;
                --web-mac) web_mac=$2 ;;
                --soak) soak=$2 ;;
                --spi-hz) spi_hz=$2 ;;
                --port) port=$2 ;;
                --out) out=$2 ;;
                --fqbn) fqbn=$2 ;;
            esac
            shift 2
            ;;
        *) die "unknown option: $1" ;;
    esac
done

[[ -n $ip ]] || die "--ip is required: a free address for the board"
valid_ipv4 "$ip" || die "--ip: not an IPv4 address: $ip"
if [[ -z $lwip_ip ]]; then
    lwip_ip=$(ip_plus "$ip" 1) || die "--lwip-ip is needed: the address after $ip is not a host address"
fi
valid_ipv4 "$lwip_ip" || die "--lwip-ip: not an IPv4 address: $lwip_ip"
[[ -z $irq || $irq =~ ^[0-9]+$ ]] || die "--irq: not a GPIO number: $irq"
[[ -z $web_mac || $web_mac =~ ^([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$ ]] || die "--web-mac: not a MAC address (aa:bb:cc:dd:ee:ff): $web_mac"
web_mac=${web_mac,,}
[[ $soak =~ ^[1-9][0-9]*$ ]] || die "--soak: not a number of seconds: $soak"
[[ -z $spi_hz || $spi_hz =~ ^[1-9][0-9]*$ ]] || die "--spi-hz: not a clock in Hz: $spi_hz"

# Say what is missing now, not after a suite has failed on it
for tool in arduino-cli python3 ping; do
    command -v "$tool" > /dev/null 2>&1 || die "$tool not found"
done
if [[ -n $web_mac ]]; then
    for tool in curl ip; do
        command -v "$tool" > /dev/null 2>&1 || die "$tool not found (the web suites need it to find the board)"
    done
fi
python3 -c 'import serial' 2> /dev/null || die "pyserial is required: pip install pyserial"
if [[ -z $port ]]; then
    shopt -s nullglob
    ports=(/dev/serial/by-id/*Pico*)
    shopt -u nullglob
    ((${#ports[@]} > 0)) || die "no Pico found in /dev/serial/by-id: plug it in, or pass --port"
    # The suites flash the board: with several Picos connected, picking one would be a guess
    ((${#ports[@]} == 1)) || die "${#ports[@]} Picos are connected (${ports[*]##*/}): pass --port with the one wired to the ENC28J60, so that no other board is flashed"
    port=${ports[0]}
fi

bench_spi=()
lwip_spi=()
if [[ -n $spi_hz ]]; then
    # --spi-assert-mhz is how fast the spi test needs the clocks to be clean: the clock under test
    spi_mhz=$(awk -v hz="$spi_hz" 'BEGIN { printf "%g", hz / 1e6 }')
    bench_spi=(--spi-hz "$spi_hz" --spi-assert-mhz "$spi_mhz")
    lwip_spi=(--spi-hz "$spi_hz")
fi
prefix=${ip%.*}

mkdir -p "$out" || die "cannot create $out"
summary=$out/summary.txt
# Only what this script writes: --out may be a folder with other things in it
rm -f "$summary" "$out"/{bench_poll,bench_cost,bench_irq,lwip_poll,lwip_irq,web_poll,web_irq}.txt
sha=$(git -C "$repo" rev-parse --short HEAD 2> /dev/null || echo unknown)
dirty=$(git -C "$repo" status --short 2> /dev/null | wc -l | tr -d ' ')

say() { # say LINE: add a line to the summary, and show it
    printf '%s\n' "$1" | tee -a "$summary"
}

status=0
suite() { # suite NAME COMMAND...: run a suite into its log, and add its line to the summary
    local name=$1 log=$out/$1.txt rc=0 line
    shift
    echo "gate: running $name, log in $log" >&2
    "$@" > "$log" 2>&1 < /dev/null || rc=$?
    line=$(grep 'passed' "$log" | tail -n 1)
    if [[ -z $line ]]; then
        line="no line with \"passed\"; the last line is: $(tail -n 1 "$log")"
    fi
    say "$name rc=$rc :: $line"
    if [[ $rc -ne 0 ]]; then
        status=1
    fi
}

skip() { # skip NAME REASON
    say "$1 skipped :: $2"
}

find_web() { # find_web: print the address that the board with the MAC address $web_mac has now
    local attempt found
    for attempt in $(seq 1 12); do
        sleep 5 # the board boots and asks DHCP for an address after the upload
        seq 1 254 | xargs -P 64 -I{} ping -c 1 -W 1 "$prefix.{}" > /dev/null 2>&1 || true
        found=$(ip -4 neigh show | awk -v mac="$web_mac" -v net="$prefix." \
            'index($1, net) == 1 && tolower($0) ~ tolower(mac) { print $1; exit }')
        if [[ -n $found ]] && curl -s -m 3 -o /dev/null "http://$found/status"; then
            echo "$found"
            return 0
        fi
        echo "attempt $attempt of 12: the board with MAC $web_mac does not answer yet" >&2
    done
    return 1
}

web_suite() { # web_suite poll|irq: build the web example, flash it, find it on the LAN, test it
    local variant=$1 build=$out/build-web-$1 found flags=()
    if [[ $variant == irq ]]; then
        flags=(--build-property "compiler.cpp.extra_flags=-DINT_PIN=$irq")
    fi
    arduino-cli compile --fqbn "$fqbn" --library "$repo" --build-path "$build" \
        ${flags[@]+"${flags[@]}"} "$repo/examples/WebServerLedToggle" || return 3
    # The uploader of the core needs the device node, not the link in /dev/serial/by-id
    arduino-cli upload -p "$(readlink -f "$port")" --fqbn "$fqbn" --input-dir "$build" || return 4
    found=$(find_web) || {
        echo "web example with MAC $web_mac not found on $prefix.0/24"
        return 5
    }
    echo "web example ($variant) at $found, commit $sha"
    python3 "$repo/extras/bench/webtest.py" --ip "$found" --soak "$soak"
}

# So that a log can be followed while its suite runs
export PYTHONUNBUFFERED=1
: > "$summary"
say "gate on $sha $dirty uncommitted paths"
echo "gate: $repo, board at $ip on $port, logs in $out" >&2

for kind in bench lwip web; do
    for variant in poll irq; do
        name=${kind}_$variant
        extra=()
        if [[ $variant == irq ]]; then
            if [[ -z $irq ]]; then
                skip "$name" "no --irq"
                continue
            fi
            extra=(--irq "$irq")
        fi
        case $kind in
            bench)
                suite "$name" python3 "$repo/extras/bench/bench.py" --ip "$ip" --flash --port "$port" --fqbn "$fqbn" \
                    ${extra[@]+"${extra[@]}"} ${bench_spi[@]+"${bench_spi[@]}"}
                # The counters of the driver are in a build made for them, which is not the build that users get: the
                # cost test runs there and nowhere else, right after the polled suite
                if [[ $variant == poll ]]; then
                    suite bench_cost python3 "$repo/extras/bench/bench.py" --ip "$ip" --flash --profile --tests cost \
                        --port "$port" --fqbn "$fqbn" ${bench_spi[@]+"${bench_spi[@]}"}
                fi
                ;;
            lwip)
                suite "$name" python3 "$repo/extras/bench/benchlwip.py" --flash --static-ip "$lwip_ip" --port "$port" --fqbn "$fqbn" \
                    ${extra[@]+"${extra[@]}"} ${lwip_spi[@]+"${lwip_spi[@]}"}
                ;;
            web)
                if [[ -z $web_mac ]]; then
                    skip "$name" "no --web-mac"
                elif [[ -n $spi_hz ]]; then
                    skip "$name" "--spi-hz is set, and the web example has no such option"
                else
                    suite "$name" web_suite "$variant"
                fi
                ;;
        esac
    done
done

say DONE
exit "$status"
