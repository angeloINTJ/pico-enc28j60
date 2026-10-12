#!/usr/bin/env python3
# Copyright 2026 Ângelo Moisés Alves
# Licensed under the MIT license, see the LICENSE file for details.
"""Simulate the SPI program of the PIO (src/enc28j60-spi.pio) cycle by cycle, and check its
timing against the data sheet of the ENC28J60.

    python3 extras/tools/pio-sim.py                  the report: bits on the wire, bytes read back, times
    python3 extras/tools/pio-sim.py --check          the same as a check: exit 1, one line per limit not met
    python3 extras/tools/pio-sim.py --check x.pio    check another program (to see the check fail on a broken one)

Run it after any change to the .pio file, then regenerate the header with
extras/tools/regenerate-pio-header.sh. The CI runs the check.

What it models. The simulator runs on the words that pioasm makes of the .pio file, so it is
the assembled program that is checked, not what its comments say. It knows the instructions
the program uses (JMP on X and Y, OUT, IN, PULL block, SET, NOP, one side-set pin, delay) and
stops with an error at any other. The pins are the driver's: SCK is the side-set pin, MOSI the
OUT pin, CS the SET pin, MISO the IN pin. The state machine is set up as start_sm() in
src/enc28j60-driver.c does it: shift left, autopull and autopush at 8 bits, MISO read without the
input synchronizer. The commands are in the TX FIFO from the start, as the driver writes them
(the bits to send, the bits to read, the command, the payload), so each command follows the one
before as soon as the program lets it: that is the shortest TCSD. The chip is ideal: it takes the
bits of MOSI at the rising edges of SCK and, once it has the command, puts the bits of its answer
on MISO at the falling edges, with no delay. The CPU keeps up, so the FIFOs never stall the state
machine, except for the TX FIFO when the commands are used up.

What it does not model. The PIO of the Pico divides the system clock by a fraction, so the state
machine steps on whole cycles of the system clock and an interval of the program is up to one of
them (8 ns at 125 MHz) shorter or longer than its nominal length: some SPI periods are shorter or
longer than 4 PIO cycles, see ENC28J60_SPI_HZ_DEFAULT in src/pico/enc28j60-driver.h. The check is
at the nominal clock and cannot see that. Nor does it see the analog side: the rise and fall
times, the time the chip takes to put a bit on MISO, the setup and hold of the data around SCK.
A pass says that the program meets the limits below at the nominal clock; it does not say that
a board works at that clock.

The check simulates at the PIO clock of the fastest SPI clock that the driver allows
(ENC28J60_SPI_HZ_MAX in src/pico/enc28j60-driver.h, read from there; --spi-mhz replaces it). Each
kind of command goes alone, and then all of them go back to back. It fails unless the bits on the
wire and the bytes read back are the expected ones and the times meet the limits of LIMITS.

Exit status: 0 if the program meets the limits (and always for the report), 1 if --check finds
something that is not met, 2 if the check could not be made (no pioasm, an instruction that the
model does not know).
"""
import argparse
import os
import re
import subprocess
import sys
import traceback
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
PROGRAM = REPO / "src" / "enc28j60-spi.pio"
DRIVER_H = REPO / "src" / "pico" / "enc28j60-driver.h"
DRIVER_C = REPO / "src" / "enc28j60-driver.c"

# The SPI timing that the ENC28J60 asks for, table 16-6 of the data sheet (DS39662E), in ns. TCSH
# is 10 ns after an access to the ETH registers and 210 ns after one to the MAC and MII registers;
# the program has a single hold for all of them, so it is held to the larger.
LIMITS = {
    "TCSS": 50,  # CS setup: CS falls to the first rising edge of SCK
    "TCSH": 210,  # CS hold: the last falling edge of SCK to CS rises
    "TCSD": 50,  # CS disable: CS is high between two commands
    "SCK period": 50,  # a high and the low after it, rising edge to rising edge: 20 MHz at most
}

# The state machine as start_sm() in src/enc28j60-driver.c sets it up
THRESHOLD = 8  # autopull and autopush, in bits
MASK32 = 0xFFFFFFFF


class CannotCheck(Exception):
    """The check could not be made: the fault is in the tools or the program, not in its timing."""


class Command:
    """One command of the driver: what is sent, how many bits are read, and what the chip answers."""

    def __init__(self, name, tx, rx_bits, answer):
        self.name = name
        self.tx = bytes(tx)  # the command byte and its payload
        self.rx_bits = rx_bits
        self.answer = bytes(answer)

    def words(self):
        """What the driver writes to the TX FIFO. A byte written to the FIFO register is repeated in
        the four lanes of the word, and the program shifts out the top one."""
        return [b * 0x01010101 for b in (8 * len(self.tx), self.rx_bits) + tuple(self.tx)]

    def sent_bits(self):
        return "".join("{:08b}".format(b) for b in self.tx)


def pattern(n, seed):
    """n bytes that are not all alike, for payloads and answers"""
    return bytes((i * 37 + seed) & 0xFF for i in range(n))


# The kinds of command of the driver (src/enc28j60-cmd.h). The buffer commands have the most bytes
# that one command can have: the program takes the sizes in bits in one byte each.
CASES = [
    Command("ETH register read", [0x1F], 8, [0xA5]),
    Command("MAC register read", [0x00], 16, [0xE1, 0x3C]),  # the first byte is the dummy one
    Command("register write", [0x5F, 0x81], 0, []),
    Command("buffer read", [0x3A], 8 * 31, pattern(31, 11)),
    Command("buffer write", bytes([0x7A]) + pattern(30, 5), 0, []),
]


class Chip:
    """The ENC28J60 as the model needs it, see the docstring of the module."""

    def __init__(self, commands):
        self.commands = commands
        self.index = -1  # the command in progress: the falls of CS count them
        self.falls = 0  # the falls of SCK since CS fell

    def pin(self, name, level):
        if name == "cs" and level == 0:
            self.index += 1
            self.falls = 0
        elif name == "sck" and level == 0:
            self.falls += 1

    def miso(self):
        if not 0 <= self.index < len(self.commands):
            return 0
        cmd = self.commands[self.index]
        # The fall that comes with the last bit of the command puts out the first bit of the answer
        k = self.falls - 8 * len(cmd.tx)
        if not 0 <= k < 8 * len(cmd.answer):
            return 0
        return (cmd.answer[k // 8] >> (7 - k % 8)) & 1


class Sim:
    """The state machine. One step is one PIO cycle, at a clock divider of 1."""

    def __init__(self, prog, side_opt, chip):
        self.prog = prog
        self.side_opt = side_opt
        self.chip = chip
        self.pc = 0
        self.x = self.y = 0
        self.osr, self.osr_count = 0, 32  # empty, so that the first OUT pulls
        self.isr, self.isr_count = 0, 0
        self.tx_fifo = []
        self.pins = {"cs": 1, "sck": 0, "mosi": 0}  # idle: CS high, SCK and MOSI low
        self.t = 0
        self.log = []  # (cycle, "cs" | "sck" | "mosi", new level), or (cycle, "rx", byte pushed)
        self.mosi_at_rise = []  # (cycle, level of MOSI) for each rising edge of SCK: what the chip takes

    def set_pin(self, name, level):
        if self.pins[name] != level:
            if name == "sck" and level == 1:
                self.mosi_at_rise.append((self.t, self.pins["mosi"]))
            self.pins[name] = level
            self.log.append((self.t, name, level))
            self.chip.pin(name, level)

    def apply_side(self, side):
        if side is not None:
            self.set_pin("sck", side)

    def side_delay(self, ins):
        """The side-set value (None if the instruction leaves the pin alone) and the delay"""
        field = (ins >> 8) & 0x1F
        if self.side_opt:  # the top bit says whether the side-set bit is there
            return ((field >> 3) & 1 if field & 0x10 else None), field & 0x7
        return field >> 4, field & 0xF

    def step(self):
        """Run the instruction at pc. A stall takes a cycle, and the instruction is tried again."""
        ins = self.prog[self.pc]
        op = ins >> 13
        side, delay = self.side_delay(ins)
        nextpc = (self.pc + 1) % len(self.prog)  # the program wraps from its end to its start
        stalled = False

        if op == 0:  # JMP
            cond, addr = (ins >> 5) & 7, ins & 0x1F
            take = True
            if cond == 1:  # !x
                take = self.x == 0
            elif cond == 2:  # x--, which counts down whether it jumps or not
                take = self.x != 0
                self.x = (self.x - 1) & MASK32
            elif cond == 3:  # !y
                take = self.y == 0
            elif cond == 4:  # y--
                take = self.y != 0
                self.y = (self.y - 1) & MASK32
            elif cond != 0:
                raise CannotCheck("instruction %04x: the jump condition %d is not modeled" % (ins, cond))
            self.apply_side(side)
            if take:
                nextpc = addr
        elif op == 3:  # OUT
            dest, count = (ins >> 5) & 7, ins & 0x1F or 32
            if self.osr_count >= THRESHOLD:  # autopull: the OSR is empty, refill it first
                if self.tx_fifo:
                    self.osr, self.osr_count = self.tx_fifo.pop(0), 0
                else:
                    stalled = True
            if not stalled:
                self.apply_side(side)
                bits = self.osr >> (32 - count)  # the top bits go first
                self.osr = (self.osr << count) & MASK32
                self.osr_count += count
                if dest == 0 and count == 1:  # pins: MOSI is the one OUT pin
                    self.set_pin("mosi", bits)
                elif dest == 1:
                    self.x = bits
                elif dest == 2:
                    self.y = bits
                elif dest != 3:  # 3 is null
                    raise CannotCheck("instruction %04x: this OUT is not modeled" % ins)
        elif op == 2:  # IN
            src, count = (ins >> 5) & 7, ins & 0x1F or 32
            self.apply_side(side)
            if src == 0 and count == 1:  # pins: MISO is the one IN pin
                bit = self.chip.miso()
            elif src == 3:  # null
                bit = 0
            else:
                raise CannotCheck("instruction %04x: this IN is not modeled" % ins)
            self.isr = ((self.isr << count) | bit) & MASK32
            self.isr_count += count
            if self.isr_count >= THRESHOLD:  # autopush
                self.log.append((self.t, "rx", self.isr & 0xFF))
                self.isr, self.isr_count = 0, 0
        elif op == 4:  # PUSH and PULL
            if (ins & 0xE0) != 0xA0:
                raise CannotCheck("instruction %04x: only 'pull block' is modeled" % ins)
            if self.tx_fifo:
                self.apply_side(side)
                self.osr, self.osr_count = self.tx_fifo.pop(0), 0
            else:
                stalled = True
        elif op == 7:  # SET
            dest, data = (ins >> 5) & 7, ins & 0x1F
            self.apply_side(side)
            if dest == 0 and data <= 1:  # pins: CS is the one SET pin
                self.set_pin("cs", data)
            elif dest == 1:
                self.x = data
            elif dest == 2:
                self.y = data
            else:
                raise CannotCheck("instruction %04x: this SET is not modeled" % ins)
        elif op == 5 and (ins & 0xE0FF) == 0xA042:  # MOV Y, Y, which is what NOP assembles to
            self.apply_side(side)
        else:
            raise CannotCheck("instruction %04x: the opcode %d is not modeled" % (ins, op))

        if stalled:
            # The side-set of a stalled instruction is set on its first cycle, and the delay comes after
            # the stall. Only a program that runs the TX FIFO dry in the middle of a command stalls here,
            # and the check reports that as a command that does not finish.
            self.apply_side(side)
            self.t += 1
        else:
            self.pc = nextpc
            self.t += 1 + delay


def simulate(prog, side_opt, commands):
    """Run the commands back to back, from a TX FIFO that holds all of them. Returns (sim, finished)."""
    sim = Sim(prog, side_opt, Chip(commands))
    for cmd in commands:
        sim.tx_fifo += cmd.words()
    # Far more than a command takes (about 4 cycles a bit): this only stops a loop that never ends
    budget = sum(32 * (8 * len(c.tx) + c.rx_bits) + 500 for c in commands)
    done = 0
    while done < len(commands) and sim.t <= budget:
        cs = sim.pins["cs"]
        sim.step()
        done += cs == 0 and sim.pins["cs"] == 1
    return sim, done == len(commands)


def first_difference(got, want):
    """Where two sequences first differ, or None if they do not"""
    for i, (a, b) in enumerate(zip(got, want)):
        if a != b:
            return i
    return None if len(got) == len(want) else min(len(got), len(want))


class Measure:
    """What a command did on the pins, from the fall of CS to its rise. The times are in ns, the
    lengths of the clock phases in cycles."""

    def __init__(self, sim, cmd, t0, t1, ns):
        self.cmd = cmd
        sck = [(t, level) for t, name, level in sim.log if name == "sck" and t0 <= t <= t1]
        rises = [t for t, level in sck if level == 1]
        self.clocks = len(rises)
        self.wire = "".join(str(level) for t, level in sim.mosi_at_rise if t0 <= t <= t1)
        self.rx = bytes(v for t, name, v in sim.log if name == "rx" and t0 <= t <= t1)
        self.cs_low = (t1 - t0) * ns
        self.tcss = (rises[0] - t0) * ns if rises else None
        self.tcsh = (t1 - sck[-1][0]) * ns if sck and sck[-1][1] == 0 else None
        self.period = min(b - a for a, b in zip(rises, rises[1:])) * ns if len(rises) > 1 else None
        self.faults = []  # SCK not low when CS moves: the times above would mean nothing
        if [level for t, name, level in sim.log if name == "sck" and t < t0][-1:] == [1]:
            self.faults.append("SCK is high when CS falls")
        if sck and sck[-1][1] == 1:
            self.faults.append("SCK is high when CS rises")
        phases = [(b[0] - a[0], a[1]) for a, b in zip(sck, sck[1:])]  # (length, level)
        self.high = self.min_max([n for n, level in phases if level == 1])
        self.low = self.min_max([n for n, level in phases if level == 0])

    @staticmethod
    def min_max(lengths):
        return (min(lengths), max(lengths)) if lengths else None

    def wire_problems(self):
        sent, found = self.cmd.sent_bits(), list(self.faults)
        if self.clocks != len(sent) + self.cmd.rx_bits:
            found.append("SCK rose %d times, expected %d" % (self.clocks, len(sent) + self.cmd.rx_bits))
        else:
            i = first_difference(self.wire[: len(sent)], sent)
            if i is not None:
                found.append("bit %d on the wire is %s, expected %s" % (i, self.wire[i], sent[i]))
        return found

    def rx_problems(self):
        got, want = self.rx, self.cmd.answer
        if len(got) != len(want):
            return ["the number of bytes read back is %d, expected %d" % (len(got), len(want))]
        i = first_difference(got, want)
        return [] if i is None else ["byte %d read back is 0x%02x, expected 0x%02x" % (i, got[i], want[i])]


class Run:
    """A case: commands simulated one after the other, and measured."""

    def __init__(self, name, commands, prog, side_opt, ns):
        self.name = name
        self.commands = commands
        sim, finished = simulate(prog, side_opt, commands)
        cs = [(t, level) for t, n, level in sim.log if n == "cs"]
        falls, rises = [t for t, level in cs if level == 0], [t for t, level in cs if level == 1]
        self.measures = [Measure(sim, c, t0, t1, ns) for c, t0, t1 in zip(commands, falls, rises)]
        self.tcsd = [(b - a) * ns for a, b in zip(rises, falls[1:])]  # CS high between two commands
        self.unfinished = None if finished else "the commands did not finish in %d PIO cycles" % sim.t

    def smallest(self, quantity):
        """The smallest value of a quantity of LIMITS in the case, or None if it was not measured"""
        if quantity == "TCSD":
            values = self.tcsd
        else:
            attr = {"TCSS": "tcss", "TCSH": "tcsh", "SCK period": "period"}[quantity]
            values = [getattr(m, attr) for m in self.measures if getattr(m, attr) is not None]
        return min(values) if values else None

    def violations(self):
        """One line for each thing that the case does not meet: first the data, then the limits"""
        found = [self.unfinished] if self.unfinished else []
        for k, m in enumerate(self.measures):
            who = "command %d (%s): " % (k + 1, m.cmd.name) if len(self.measures) > 1 else ""
            found += [who + text for text in m.wire_problems() + m.rx_problems()]
        for quantity, limit in LIMITS.items():
            value = self.smallest(quantity)
            if value is not None and value < limit - 1e-6:
                found.append("%s %.1f ns, the limit is %g ns" % (quantity, value, limit))
        return ["%s: %s" % (self.name, text) for text in found]


def find_pioasm(explicit):
    """The same rule as extras/tools/regenerate-pio-header.sh: the one asked for, else $PIOASM, else
    the last one, by name, under the data folder of Arduino"""
    found = explicit or os.environ.get("PIOASM")
    if found:
        return found
    data = Path(os.environ.get("ARDUINO_DATA") or Path.home() / ".arduino15")
    candidates = sorted(str(p) for p in (data / "packages/rp2040/tools/pqt-pioasm").glob("**/pioasm") if p.is_file())
    if not candidates:
        raise CannotCheck("pioasm not found: install the rp2040 core (arduino-cli core install rp2040:rp2040), "
                          "or set PIOASM (or use --pioasm) to the path of one")
    return candidates[-1]


def assemble(program, pioasm):
    """The words that pioasm makes of the .pio file, and whether its side-set bit is optional"""
    try:
        source = program.read_text(encoding="utf-8")
        made = subprocess.run([pioasm, "-o", "hex", str(program)], capture_output=True, text=True)
    except OSError as e:
        raise CannotCheck("%s: %s" % (e.filename or pioasm, e.strerror))
    if made.returncode != 0:
        raise CannotCheck("pioasm did not assemble %s:\n%s" % (program, (made.stderr or made.stdout).strip()))
    side = re.search(r"^[ \t]*\.side_set[ \t]+(\d+)[ \t]*(opt)?[ \t]*(pindirs)?", source, re.M)
    if not side or side.group(1) != "1" or side.group(3):
        raise CannotCheck("the model needs '.side_set 1' or '.side_set 1 opt': SCK is its only side-set pin")
    if re.search(r"^[ \t]*\.wrap", source, re.M):
        raise CannotCheck("the model does not do .wrap or .wrap_target: the program wraps from its end to its start")
    return [int(word, 16) for word in made.stdout.split()], bool(side.group(2))


def read_define(path, name):
    """The number in '#define name <number>' of a source file of the driver"""
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as e:
        raise CannotCheck("%s: %s" % (path, e.strerror))
    found = re.search(r"^[ \t]*#[ \t]*define[ \t]+%s[ \t]+(\d+)[uUlL]*\b" % re.escape(name), text, re.M)
    if not found:
        raise CannotCheck("%s: no '#define %s <number>'" % (path, name))
    return int(found.group(1))


def shown(path):
    """A path relative to the repository if it is inside it"""
    try:
        return str(path.resolve().relative_to(REPO))
    except ValueError:
        return str(path)


def hexes(data, n=8):
    """The first bytes of a sequence, for the report"""
    if not data:
        return "none"
    return " ".join("0x%02x" % b for b in data[:n]) + (" ... (%d bytes)" % len(data) if len(data) > n else "")


def report(run):
    """The numbers of a case, as the original simulator printed them, with the limits added"""
    if len(run.measures) == 1:
        cmd = run.commands[0]
        print("\n== %s (%d tx bits, %d rx bits)" % (cmd.name, 8 * len(cmd.tx), cmd.rx_bits))
    else:
        print("\n== %s (%d commands, the next one already in the TX FIFO)" % (run.name, len(run.commands)))
    if run.unfinished:
        print("   ! " + run.unfinished)
    for k, m in enumerate(run.measures):
        wire, rx = m.wire_problems(), m.rx_problems()
        if len(run.measures) == 1:
            sent = m.cmd.sent_bits()
            print("   tx bits on the wire %s: %s" % ("WRONG" if wire else "OK", sent if len(sent) <= 64 else sent[:64] + "..."))
            expected = " (expected %s)" % hexes(m.cmd.answer) if rx else ""
            print("   rx bytes %s: %s%s" % ("WRONG" if rx else "OK", hexes(m.rx), expected))
            rows = [("clocks", m.clocks),
                    ("tcss_ns (limit %g)" % LIMITS["TCSS"], m.tcss),
                    ("tcsh_ns, last SCK fall -> CS rise (limit %g)" % LIMITS["TCSH"], m.tcsh),
                    ("sck_high_cycles (min..max)", m.high),
                    ("sck_low_cycles (min..max)", m.low),
                    ("sck_period_ns, smallest (limit %g)" % LIMITS["SCK period"], m.period),
                    ("cs_low_ns_total", m.cs_low)]
            for label, value in rows:
                print("   %-46s %s" % (label, value))
        elif wire or rx:
            print("   command %d (%s): bits or bytes WRONG" % (k + 1, m.cmd.name))
        for text in wire + rx:
            print("   ! " + text)
    if len(run.measures) > 1:
        if not any(m.wire_problems() + m.rx_problems() for m in run.measures):
            print("   tx bits on the wire OK and rx bytes OK, in every command")
        print("   %-46s %.1f" % ("tcss_ns, smallest (limit %g)" % LIMITS["TCSS"], run.smallest("TCSS")))
        print("   %-46s %.1f" % ("tcsh_ns, smallest (limit %g)" % LIMITS["TCSH"], run.smallest("TCSH")))
        print("   %-46s %.1f" % ("sck_period_ns, smallest (limit %g)" % LIMITS["SCK period"], run.smallest("SCK period")))
        print("   tcsd_ns, CS high between commands (limit %g): %s" % (
            LIMITS["TCSD"], ", ".join("%.1f" % g for g in run.tcsd)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("program", nargs="?", type=Path, default=PROGRAM,
                    help="the .pio file to simulate (default: src/enc28j60-spi.pio of this repository)")
    ap.add_argument("--check", action="store_true",
                    help="check instead of report: a line for each problem and exit 1, or one line and exit 0")
    ap.add_argument("--spi-mhz", type=float, metavar="MHZ",
                    help="the SPI clock to simulate at (default: ENC28J60_SPI_HZ_MAX of the driver)")
    ap.add_argument("--pioasm", metavar="PATH",
                    help="the pioasm to use (default: the rule of extras/tools/regenerate-pio-header.sh)")
    args = ap.parse_args()
    if args.spi_mhz is not None and args.spi_mhz <= 0:
        ap.error("--spi-mhz must be above 0")

    try:
        spi_mhz = args.spi_mhz or read_define(DRIVER_H, "ENC28J60_SPI_HZ_MAX") / 1e6
        pio_mhz = spi_mhz * read_define(DRIVER_C, "SPI_PIO_CLOCKS_PER_BIT")
        pioasm = find_pioasm(args.pioasm)
        prog, side_opt = assemble(args.program, pioasm)
        ns = 1000.0 / pio_mhz
        runs = [Run(c.name, [c], prog, side_opt, ns) for c in CASES]
        runs.append(Run("back to back", CASES * 2, prog, side_opt, ns))
    except CannotCheck as e:
        print("pio-sim: %s" % e, file=sys.stderr)
        return 2

    clock = "SPI %g MHz (PIO %g MHz, %g ns a cycle)" % (spi_mhz, pio_mhz, ns)
    if not args.check:
        print("program: %s, %d instructions, assembled by %s" % (shown(args.program), len(prog), pioasm))
        print("clock: %s" % clock)
        for run in runs:
            report(run)
        return 0

    lines = [line for run in runs for line in run.violations()]
    for line in lines:
        print("FAIL " + line)
    if lines:
        return 1
    print("ok: %s at %s: %d cases meet table 16-6 of the data sheet. Smallest values: %s" % (
        shown(args.program), clock, len(runs),
        ", ".join("%s %.1f ns (limit %g)" % (q, min(v for v in (r.smallest(q) for r in runs) if v is not None), limit)
                  for q, limit in LIMITS.items())))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception:  # a fault of the tool is not a finding about the program: 1 is for the findings
        traceback.print_exc()
        sys.exit(2)
