// duplex_wire_dsp.c  —  capture -> int32 DSP hook -> pack -> playback
#define _GNU_SOURCE
#include <alsa/asoundlib.h>
#include <pthread.h>
#include <sys/mman.h>
#include <sched.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>

typedef struct {
    const char *name;
    snd_pcm_t *pcm;
    unsigned int rate;
    snd_pcm_format_t format;
    snd_pcm_access_t access;
    unsigned int channels;
    snd_pcm_uframes_t period;
    unsigned int periods;
} pcm_side_t;

static void make_realtime(void) {
    struct sched_param sp = { .sched_priority = 70 };
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
        fprintf(stderr, "mlockall: %s\n", strerror(errno));
    int e = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    if (e) fprintf(stderr, "SCHED_FIFO: %s (need cap_sys_nice)\n", strerror(e));
}

static void set_hw_sw_params(pcm_side_t *s) {
    int err;
    snd_pcm_hw_params_t *hw; snd_pcm_sw_params_t *sw;
    snd_pcm_hw_params_alloca(&hw);
    snd_pcm_sw_params_alloca(&sw);

    snd_pcm_hw_params_any(s->pcm, hw);
    if ((err = snd_pcm_hw_params_set_access(s->pcm, hw, s->access)) < 0)
        fprintf(stderr, "[%s] set_access: %s\n", s->name, snd_strerror(err));

    if ((err = snd_pcm_hw_params_set_format(s->pcm, hw, s->format)) < 0) {
        if (snd_pcm_hw_params_set_format(s->pcm, hw, SND_PCM_FORMAT_S32_LE) == 0) {
            fprintf(stderr, "[%s] Info: switching format to supported S32_LE\n", s->name);
            s->format = SND_PCM_FORMAT_S32_LE;
        } else if (snd_pcm_hw_params_set_format(s->pcm, hw, SND_PCM_FORMAT_S16_LE) == 0) {
            fprintf(stderr, "[%s] Info: switching format to supported S16_LE\n", s->name);
            s->format = SND_PCM_FORMAT_S16_LE;
        } else {
            fprintf(stderr, "[%s] No supported format.\n", s->name);
            exit(1);
        }
    }

    unsigned int req_rate = s->rate;
    if ((err = snd_pcm_hw_params_set_rate_near(s->pcm, hw, &s->rate, 0)) < 0) {
        fprintf(stderr, "[%s] set_rate: %s\n", s->name, snd_strerror(err));
        exit(1);
    }
    if (s->rate != req_rate)
        fprintf(stderr, "[%s] Info: switching rate to supported %u\n", s->name, s->rate);

    if ((err = snd_pcm_hw_params_set_channels(s->pcm, hw, s->channels)) < 0) {
        fprintf(stderr, "[%s] set_channels: %s\n", s->name, snd_strerror(err));
        exit(1);
    }

    if ((err = snd_pcm_hw_params_set_period_size_near(s->pcm, hw, &s->period, 0)) < 0)
        fprintf(stderr, "[%s] set_period: %s\n", s->name, snd_strerror(err));
    if ((err = snd_pcm_hw_params_set_periods_near(s->pcm, hw, &s->periods, 0)) < 0)
        fprintf(stderr, "[%s] set_periods: %s\n", s->name, snd_strerror(err));

    if ((err = snd_pcm_hw_params(s->pcm, hw)) < 0) {
        fprintf(stderr, "[%s] hw_params: %s\n", s->name, snd_strerror(err));
        exit(1);
    }

    snd_pcm_sw_params_current(s->pcm, sw);
    snd_pcm_sw_params_set_avail_min(s->pcm, sw, s->period);
    snd_pcm_sw_params_set_start_threshold(s->pcm, sw, s->period);
    if ((err = snd_pcm_sw_params(s->pcm, sw)) < 0) {
        fprintf(stderr, "[%s] sw_params: %s\n", s->name, snd_strerror(err));
        exit(1);
    }
}

static const char *fmt_name(snd_pcm_format_t f) {
    switch (f) {
        case SND_PCM_FORMAT_S16_LE: return "S16_LE";
        case SND_PCM_FORMAT_S24_LE: return "S24_LE";
        case SND_PCM_FORMAT_S32_LE: return "S32_LE";
        default: return "UNKNOWN";
    }
}

/* ----------------- sample conversion helpers ----------------- */
/* We use int32 PCM in the DSP core:
   - S16  -> shift left 16 (sign preserved)
   - S24_LE (24-in-32, little-endian, LSB-aligned) -> sign-extend from 24 bits
   - S32  -> passthrough
*/
static inline int32_t clamp_s16_from_i32(int32_t x) {
    if (x >  0x7FFF0000) x =  0x7FFF0000;
    if (x < (int32_t)0x80000000 >> 16 << 16) x = (int32_t)0x80000000 >> 16 << 16; // -32768<<16
    return x;
}

static void unpack_to_i32(const void *in, int32_t *out, snd_pcm_format_t fmt, size_t samples) {
    if (fmt == SND_PCM_FORMAT_S32_LE) {
        const int32_t *p = (const int32_t*)in;
        for (size_t i=0;i<samples;i++) out[i] = p[i];
    } else if (fmt == SND_PCM_FORMAT_S16_LE) {
        const int16_t *p = (const int16_t*)in;
        for (size_t i=0;i<samples;i++) out[i] = ((int32_t)p[i]) << 16;
    } else if (fmt == SND_PCM_FORMAT_S24_LE) {
        const int32_t *p = (const int32_t*)in; // 24 bits in 32, LSB-aligned
        for (size_t i=0;i<samples;i++) {
            int32_t v = p[i] << 8;  // move sign bit to bit 31
            out[i] = v >> 8;        // arithmetic shift back: sign-extended 24 -> 32
        }
    }
}

static void pack_from_i32(const int32_t *in, void *out, snd_pcm_format_t fmt, size_t samples) {
    if (fmt == SND_PCM_FORMAT_S32_LE) {
        int32_t *q = (int32_t*)out;
        for (size_t i=0;i<samples;i++) q[i] = in[i];
    } else if (fmt == SND_PCM_FORMAT_S16_LE) {
        int16_t *q = (int16_t*)out;
        for (size_t i=0;i<samples;i++) q[i] = (int16_t)(in[i] >> 16);
    } else if (fmt == SND_PCM_FORMAT_S24_LE) {
        int32_t *q = (int32_t*)out; // store sign-extended value with lower 24 bits significant
        for (size_t i=0;i<samples;i++) {
            int32_t v = (in[i] << 8) >> 8; // keep 24-bit range with sign
            q[i] = v;
        }
    }
}

/* -------------- tiny example DSP hook (no-op / gain) ---------- */
/* Replace this with your own DSP. int32 full-scale depends on input format as prepared above. */
static void dsp_process_int32(int32_t *samples, size_t n, unsigned channels) {
    (void)channels;
    // Example: unity gain (no-op). For a quick test, try 0.5 gain:
    // for (size_t i=0;i<n;i++) samples[i] = samples[i] / 10;
}

/* -------------------------------------------------------------- */

static void open_side(pcm_side_t *s, const char *dev, int stream) {
    int err = snd_pcm_open(&s->pcm, dev, stream, 0);
    if (err < 0) {
        fprintf(stderr, "[%s] open %s: %s\n", s->name, dev, snd_strerror(err));
        exit(1);
    }
}

static void usage(const char *p) {
    fprintf(stderr,
        "Usage: %s [--cap hw:X,Y] [--play hw:A,B] [--rate 48000] [--ch 2]\n"
        "          [--period 128] [--periods 3] [--format S32_LE|S24_LE|S16_LE]\n", p);
}

int main(int argc, char **argv) {
    const char *cap_dev = "hw:1,0";
    const char *play_dev = "hw:1,0";
    unsigned int rate = 48000, ch = 2, periods = 3;
    snd_pcm_uframes_t period = 128;
    snd_pcm_format_t format = SND_PCM_FORMAT_S32_LE;

    for (int i=1;i<argc;i++) {
        if (!strcmp(argv[i],"--cap") && i+1<argc) cap_dev = argv[++i];
        else if (!strcmp(argv[i],"--play") && i+1<argc) play_dev = argv[++i];
        else if (!strcmp(argv[i],"--rate") && i+1<argc) rate = atoi(argv[++i]);
        else if (!strcmp(argv[i],"--ch") && i+1<argc) ch = atoi(argv[++i]);
        else if (!strcmp(argv[i],"--period") && i+1<argc) period = atoi(argv[++i]);
        else if (!strcmp(argv[i],"--periods") && i+1<argc) periods = atoi(argv[++i]);
        else if (!strcmp(argv[i],"--format") && i+1<argc) {
            const char *f = argv[++i];
            if (!strcmp(f,"S16_LE")) format = SND_PCM_FORMAT_S16_LE;
            else if (!strcmp(f,"S24_LE")) format = SND_PCM_FORMAT_S24_LE;
            else if (!strcmp(f,"S32_LE")) format = SND_PCM_FORMAT_S32_LE;
        } else { usage(argv[0]); return 1; }
    }

    make_realtime();

    pcm_side_t cap = {.name="CAP", .rate=rate, .format=format, .access=SND_PCM_ACCESS_RW_INTERLEAVED,
                      .channels=ch, .period=period, .periods=periods};
    pcm_side_t play= {.name="PLAY",.rate=rate, .format=format, .access=SND_PCM_ACCESS_RW_INTERLEAVED,
                      .channels=ch, .period=period, .periods=periods};

    open_side(&cap, cap_dev, SND_PCM_STREAM_CAPTURE);
    open_side(&play, play_dev, SND_PCM_STREAM_PLAYBACK);

    set_hw_sw_params(&cap);
    set_hw_sw_params(&play);

    fprintf(stderr, "Negotiated:\n  CAP : %s %u Hz ch=%u period=%lu periods=%u\n",
            fmt_name(cap.format), cap.rate, cap.channels,
            (unsigned long)cap.period, cap.periods);
    fprintf(stderr, "  PLAY: %s %u Hz ch=%u period=%lu periods=%u\n",
            fmt_name(play.format), play.rate, play.channels,
            (unsigned long)play.period, play.periods);

    if (snd_pcm_link(cap.pcm, play.pcm) == 0) {
        fprintf(stderr, "Linked CAP & PLAY for synchronous start.\n");
    } else {
        snd_pcm_prepare(play.pcm);
        snd_pcm_prepare(cap.pcm);
    }

    const size_t frames = cap.period;
    const size_t samples = (size_t)frames * cap.channels;
    const size_t bytes_per_sample =
        (cap.format == SND_PCM_FORMAT_S16_LE) ? 2 :
        (cap.format == SND_PCM_FORMAT_S24_LE) ? 4 : 4;
    const size_t buf_bytes = samples * bytes_per_sample;

    void *io_buf = malloc(buf_bytes);
    int32_t *dsp_buf = (int32_t*)malloc(samples * sizeof(int32_t));
    if (!io_buf || !dsp_buf) { perror("malloc"); return 1; }

    memset(io_buf, 0, buf_bytes);
    for (int i=0;i<2;i++) {
        snd_pcm_sframes_t w = snd_pcm_writei(play.pcm, io_buf, frames);
        if (w < 0) snd_pcm_recover(play.pcm, (int)w, 1);
    }

    snd_pcm_prepare(cap.pcm);
    snd_pcm_prepare(play.pcm);
    snd_pcm_start(cap.pcm);
    snd_pcm_start(play.pcm);

    for (;;) {
        snd_pcm_sframes_t r = snd_pcm_readi(cap.pcm, io_buf, frames);
        if (r < 0) {
            int e = (int)r;
            if (e == -EPIPE || e == -ESTRPIPE) {
                r = snd_pcm_recover(cap.pcm, e, 1);
                if (r < 0) { fprintf(stderr, "CAP recover failed: %s\n", snd_strerror((int)r)); break; }
                continue;
            } else { fprintf(stderr, "read error: %s\n", snd_strerror(e)); break; }
        } else if ((size_t)r < frames) {
            // zero-pad short read so unpack doesn't read garbage
            size_t want = frames - (size_t)r;
            memset((char*)io_buf + (size_t)r * bytes_per_sample * cap.channels, 0,
                   want * bytes_per_sample * cap.channels);
        }

        // --- Convert bytes -> int32
        unpack_to_i32(io_buf, dsp_buf, cap.format, samples);

        // --- DSP hook (edit here)
        dsp_process_int32(dsp_buf, samples, cap.channels);

        // --- Convert int32 -> bytes
        pack_from_i32(dsp_buf, io_buf, play.format, samples);

        snd_pcm_sframes_t w = snd_pcm_writei(play.pcm, io_buf, frames);
        if (w < 0) {
            int e = (int)w;
            if (e == -EPIPE || e == -ESTRPIPE) {
                w = snd_pcm_recover(play.pcm, e, 1);
                if (w < 0) { fprintf(stderr, "PLAY recover failed: %s\n", snd_strerror((int)w)); break; }
                continue;
            } else { fprintf(stderr, "write error: %s\n", snd_strerror(e)); break; }
        }
    }

    snd_pcm_close(cap.pcm);
    snd_pcm_close(play.pcm);
    free(dsp_buf);
    free(io_buf);
    return 0;
}
