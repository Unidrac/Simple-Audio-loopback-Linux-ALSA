# ALSA Duplex Wire with DSP Hook

This project is a low-latency Linux C program that captures audio from an ALSA device and immediately plays it back to another (or the same) ALSA device.  
Between **capture** and **playback**, the samples are converted into signed 32-bit integers (`int32_t`) so that you can insert custom DSP code.

---

## Features

- Full duplex capture → process → playback.
- Supports `S16_LE`, `S24_LE` (packed in 32 bits), and `S32_LE`.
- Converts to/from `int32_t` samples for simple DSP integration.
- Real-time friendly:
  - Uses `SCHED_FIFO` scheduling.
  - Locks memory (`mlockall`) to prevent page faults.
  - Small ALSA period sizes (64–128 frames) for low latency.
  - Xrun (underrun/overrun) recovery.
- Works with `hw:` devices (avoids ALSA plug conversions).

---

## Build

```bash
gcc duplex_wire_dsp.c -O2 -Wall -lasound -lpthread -o duplex_wire_dsp
