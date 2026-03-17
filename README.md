# Mikrotoner

An STM32-based digital synthesizer built on a perforated PCB prototype, running on the STM32H755ZIT6 dual-core microcontroller. All DSP runs in real-time on the Cortex-M7 core, with audio output via I2S at 48kHz/32-bit.

## Demo

[▶ Mikrotoner - STM32 Synthesizer Demo](https://www.youtube.com/watch?v=cM_hrNbR9Ko)

## Features

### Core Sound Engine

- **Dual Oscillators** — continuous waveform morphing (sine → triangle → sawtooth → square) with adjustable duty cycle
- **Anti-Aliasing** — PolyBLEP algorithm smooths discontinuities in sawtooth and square waves
- **Modulation** — phase modulation (PM) and amplitude modulation (AM), with oscillator 2 modulating oscillator 1
- **Hard Sync** — oscillator 2 phase resets when oscillator 1 phase resets
- **Noise Generator** — color control (low-pass / high-pass shaping) with stereo width via two independent noise sources
- **Stereo** — individual panning controls per oscillator

### Performance & Control

- **Ribbon Control** — pitch via membrane potentiometer, amplitude via pressure sensor (velocity-like behavior)
- **Quantization** — switchable between microtonal (4095 pitch values) and semitone (24 values) modes
- **Portamento** — adjustable glide time from 0.1s to 1s

### Filters & Modulation

- **State Variable Filter** — selectable low-pass, band-pass, and high-pass with resonance and cutoff control
- **Dual LFO** — two LFOs with adjustable rate (1–20Hz) and variable waveforms (sine, triangle, sawtooth, square)
- **LFO Routing** — assignable to pitch, waveshape, and filter cutoff; LFOs can modulate each other's waveshape or frequency
- **ADSR Envelope** — assignable to amplitude, all parameters adjustable from 0.01s to 5s

## Hardware

| Component | Details |
|---|---|
| MCU | STM32H755ZIT6 (Cortex-M7 480MHz + Cortex-M4 240MHz) |
| Audio output | I2S2 (SPI2), 48kHz, 32-bit |
| ADC | ADC2 + ADC3 with DMA, 12-bit oversampled |
| Multiplexer | 16-channel analog mux for knobs and switches |
| Ribbon controller | Membrane potentiometer + pressure sensor |
| Prototype | Perforated PCB |

The Cortex-M7 handles all DSP processing, peripheral initialization, and audio output.
In future revisions, the Cortex-M4 core is planned to handle ADC processing, 
MIDI input, and LCD display, offloading these tasks from the Cortex-M7 and allowing it to focus entirely on DSP.

## Code Structure

```
Core/
├── Inc/
│   ├── dsp_adsr.h        # ADSR envelope types and function declarations
│   ├── dsp_config.h      # Global constants (sample rate, buffer size, thresholds)
│   ├── dsp_enum.h        # Enums for enable states, modulation modes, LFO selection
│   ├── dsp_filter.h      # State variable filter types and declarations
│   ├── dsp_lfo.h         # LFO types and declarations
│   ├── dsp_noise.h       # Noise generator types and declarations
│   ├── dsp_osc.h         # Oscillator types and declarations
│   └── dsp_utils.h       # Shared utilities (lerp, poly_blep, soft_clip, LP filter)
└── Src/
    ├── main.c            # Peripheral init, main loop, ADC/I2S callbacks
    ├── dsp_adsr.c        # ADSR envelope processing
    ├── dsp_filter.c      # State variable filter
    ├── dsp_lfo.c         # LFO generation and modulation
    ├── dsp_noise.c       # White/colored noise generation (XORshift)
    ├── dsp_osc.c         # Dual oscillator with PM/AM modulation
    └── dsp_utils.c       # Utility functions
```

## Known Issues / Roadmap

- ADC processing in main loop `default` case to be extracted into dedicated functions

## Version

`v0.1.0` — Initial commit of working perforated PCB prototype
