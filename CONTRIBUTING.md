# Contributing

Bug reports, fixes and ideas are welcome. This page says how the repository is
checked, so that a change can be checked before it is sent.

## Reporting a problem

Open an [issue](https://github.com/angeloINTJ/pico-enc28j60/issues) and fill in the
form. The three things that matter most are the board, the version of the arduino-pico
core, and how the chip is wired (the `ENC28J60Config` you pass). `lastError()` and
`stats()` say a lot about what went wrong, so include what they print.

## Changing the code

The layout of the repository is in the [README](README.md#repository-layout), and who
wrote what in [NOTICE.md](NOTICE.md). Two rules come from there:

 * The files of the original project (`src/enc28j60-*`, `src/pico/`, `freertos/`) keep
   the copyright notice of Lukasz Janyst. When you change one, add the
   "Modifications copyright" line that the files already have and list the change in
   the section of NOTICE.md that covers them.
 * What a change does to the chip has to be backed by the
   [data sheet](https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/DataSheets/39662e.pdf)
   or the [errata](https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/Errata/80349c.pdf),
   and the comment says which section or which issue.

## Checks

The [CI](.github/workflows/ci.yml) runs these on every push; each one can be run
locally from the root of the repository.

| Check | Command |
|-------|---------|
| Library metadata and examples | `arduino-lint --library-manager false --compliance strict --project-type library "$PWD"` |
| Formatting of the C and C++ sources | `extras/tools/check-format.sh`, the command that the CI runs (it needs clang-format 18, because other versions format differently; `CLANG_FORMAT` names another binary, and `clang-format-18 -i` on the files it names fixes them) |
| Links between the Markdown files | `python3 extras/tools/check-links.py` |
| The PIO header | `extras/tools/regenerate-pio-header.sh` after any change to `src/enc28j60-spi.pio`; the CI checks that the program in `src/enc28j60-spi.pio.h` is the one pioasm makes of the `.pio` file |
| The timing of the PIO program | `python3 extras/tools/pio-sim.py --check` simulates the assembled program at the fastest SPI clock the driver allows and fails unless the chip select times and the clock period of table 16-6 of the data sheet are met (`--spi-mhz` simulates another clock, and a `.pio` file as the last argument checks another program); without `--check` it prints the report |
| Build | the examples and the bench sketches for the Pico, Pico W, Pico 2 and Pico 2 W, with the oldest and the newest core the library supports, and the C driver as a CMake library with the Pico SDK (`-Werror`, and on the driver the static analyzer of GCC and `-Wconversion`) |

## Testing on hardware

The CI cannot test a network. A change to the driver, to the PIO program or to the lwIP
interface needs the tests of [`extras/bench`](extras/bench/README.md) on a real board:
`bench.py` (the raw class) and `benchlwip.py` (the lwIP interface), with and without
the INT pin if the change can tell the difference, and `webtest.py` for the web
example. [`extras/bench/gate.sh`](extras/bench/gate.sh) runs all of them, one after the
other, and writes a summary ([how](extras/bench/README.md#the-whole-gate)); the change
goes in only after it passed. Say in the pull request what you ran and what it printed. A
change that fixes a bug should come with a test that fails without it; the bench is where
such tests go. The CI looks at the tip of a branch: a branch with several commits can be
checked commit by commit, with no board, by `extras/tools/check-commits.sh <base> <branch>`
(the formatting, the links and the build of the examples and the bench sketches, for each
commit).

## Commits and pull requests

 * One change per commit, and a subject that says what it does. The body says why:
   what was wrong, how it showed, what the data sheet says.
 * Add a line to the "Unreleased" section of [CHANGELOG.md](CHANGELOG.md) for anything
   a user of the library can notice.
 * Do not claim in a commit message, a comment or the documentation what you did not
   check; say what you checked and how.
