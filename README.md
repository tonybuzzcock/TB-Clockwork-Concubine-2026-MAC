# TB Clockwork Concubine (2026 Mac)

The TB Clockwork Concubine flanger for current Macs: **CLAP, VST3 and AU**,
universal (Apple Silicon and Intel), **macOS 11 and later**. Use it in Reaper, FL Studio,
Logic, GarageBand, Tape16 and other hosts.

The sound and panel are the same as the Mac OS X 10.8 VST build
([TB-Clockwork-Concubine](https://github.com/tonybuzzcock/TB-Clockwork-Concubine)).
The DSP code is the same, and the test suite checks that the plugin's output
matches it sample for sample. It's a model of the HH Electronics Clockwork
Concubine, built from its schematic (in `Docs/`). Personal, non-commercial use.

## Install

GitHub → Actions → latest run → Artifacts → `TB-Clockwork-Concubine-mac`, unzip,
then follow `INSTALL.txt`. In short:

- `TB Clockwork Concubine.vst3` → `~/Library/Audio/Plug-Ins/VST3/`
- `TB Clockwork Concubine.component` → `~/Library/Audio/Plug-Ins/Components/`
- `TB Clockwork Concubine.clap` → `~/Library/Audio/Plug-Ins/CLAP/`

Apple hasn't notarised the plugins, so clear the quarantine flag once in Terminal (the
exact command is in `INSTALL.txt`), then rescan plugins in your DAW.

## Controls

| Name | Control | Range |
|------|---------|-------|
| Frequency | LFO rate (VR2 1M rev log) | 0.1 – 10.5 Hz |
| Depth | how far the delay sweeps (VR1) | 0 – 10 |
| Resonance | feedback (VR4) | 0 – 10 |
| Effect | the stomp switch: effect in, or straight through | In / Out |
| Input | *plugin only*: level into the pedal | ±12 dB |
| Output | *plugin only*: output trim | ±24 dB |

In stereo, each side runs through its own copy of the circuit, with the same
settings and the two sweeps in step. A mono source is processed once.

## The panel

480 × 720, the pedal's 2:3 portrait shape. At the top are the FREQUENCY, DEPTH
and RESONANCE knobs (scales marked 0 and 10). Below them are the name plate and
the striped panel with the stomp switch, which glows red with the effect in.
The strip along the bottom has the TB Electronic mark, Input, Output, presets
and the readout.

Drag a knob up or down to turn it (hold Shift for fine control). Double-click a
knob to reset it, or use the scroll wheel. Click the stomp switch to switch the
effect in or out.

**Presets:** click SAVE, then a slot 1–8. Click a saved slot to recall it. The
presets are shared by all three formats, in
`~/Library/Application Support/TB/TB Clockwork Concubine/Presets.txt`.

## What's modelled

The Q1 buffer, IC2 with its treble lift and the resonance loop, and the SAD1024
modelled as two 512-stage bucket chains clocked in antiphase, with its staircase
output and clock aliasing. Also the IC5 mixer with its de-emphasis, and the 741s
on the ±6V8 zener rails. The delay range (about 0.45 – 5.6 ms), the LFO rate range
and the SAD1024's gain and clipping aren't fixed by the drawing, so they're
estimated. Each is a named constant in `Source/DSP/Flanger.cpp`. The
10.8 build's README has the full circuit notes.

## How it's built

The plugin is written once as a CLAP (`Source/Plugin`).
[clap-wrapper](https://github.com/free-audio/clap-wrapper) v0.17.0 builds the
VST3 and AU from that same code. The SDKs are downloaded at configure time:

    cmake -B build -DCMAKE_BUILD_TYPE=Release
    cmake --build build --config Release

GitHub Actions (`macos-15`) builds universal binaries and fails unless every
plugin has both architectures and a macOS 11.0 minimum. It ad-hoc signs the
plugins, then:

- runs the DSP, preset and CLAP host tests (native and under Rosetta)
- opens the panel offscreen through the CLAP GUI extension and drives its controls
- validates with auval, clap-validator and pluginval
- publishes panel screenshots to the `gui-preview` branch

A Linux job builds the CLAP and runs the same tests.
