/*
 * tascam_test — outil de test du moteur US-122 MKII
 *
 *   tascam_test info
 *   tascam_test bip   [fréquence] [secondes]
 *   tascam_test rec   [fréquence] [secondes] [fichier.wav]
 *   tascam_test thru  [fréquence] [secondes]     (entrée -> sortie)
 *   tascam_test clock [fréquence] [secondes]     (stabilité de l'horloge)
 */
#include "tascam_usb.h"

#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

static volatile sig_atomic_t g_stop = 0;
static void on_sigint(int s) { (void)s; g_stop = 1; }

typedef struct {
    tascam_engine *e;
    int mode;            /* 1 bip, 2 rec, 3 thru, 4 clock */
    int rate;
    double phase;
    float *rec;
    size_t rec_cap, rec_len;   /* en trames */
    float peak[2];
    uint64_t thru_delay;
    double bipfreq;
} ctx_t;

static void on_input(void *vctx, const float *frames, uint32_t n, uint64_t pos)
{
    ctx_t *c = (ctx_t *)vctx;
    for (uint32_t i = 0; i < n; i++) {
        float l = fabsf(frames[i * 2]), r = fabsf(frames[i * 2 + 1]);
        if (l > c->peak[0]) c->peak[0] = l;
        if (r > c->peak[1]) c->peak[1] = r;
    }
    if (c->mode == 2) {
        if (c->rec_len + n <= c->rec_cap) {
            memcpy(c->rec + c->rec_len * 2, frames, n * 2 * sizeof(float));
            c->rec_len += n;
        }
    } else if (c->mode == 3) {
        /* l'entrée arrive en retard sur la sortie : on la place un peu plus loin */
        tascam_write_output(c->e, pos + c->thru_delay, frames, n);
    }
}

/* bip : généré au moment où le bloc de sortie part vers la carte */
static void on_output(void *vctx, float *frames, uint32_t n, uint64_t pos)
{
    ctx_t *c = (ctx_t *)vctx;
    (void)pos;
    if (c->mode != 1) return;
    for (uint32_t i = 0; i < n; i++) {
        float v = 0.2f * (float)sin(c->phase);
        c->phase += 2.0 * M_PI * c->bipfreq / c->rate;
        if (c->phase > 2.0 * M_PI) c->phase -= 2.0 * M_PI;
        frames[i * 2] = v;
        frames[i * 2 + 1] = v;
    }
}

static void put32(FILE *f, uint32_t v) { fputc(v & 255, f); fputc((v >> 8) & 255, f); fputc((v >> 16) & 255, f); fputc((v >> 24) & 255, f); }
static void put16(FILE *f, uint16_t v) { fputc(v & 255, f); fputc((v >> 8) & 255, f); }

static int write_wav(const char *path, const float *data, size_t frames, int rate)
{
    FILE *f = fopen(path, "wb");
    if (!f) return -1;
    uint32_t bytes = (uint32_t)(frames * 6);
    fwrite("RIFF", 1, 4, f); put32(f, 36 + bytes); fwrite("WAVE", 1, 4, f);
    fwrite("fmt ", 1, 4, f); put32(f, 16); put16(f, 1); put16(f, 2);
    put32(f, (uint32_t)rate); put32(f, (uint32_t)rate * 6); put16(f, 6); put16(f, 24);
    fwrite("data", 1, 4, f); put32(f, bytes);
    for (size_t i = 0; i < frames * 2; i++) {
        float v = data[i];
        if (v > 1) v = 1;
        if (v < -1) v = -1;
        int32_t s = (int32_t)lrintf(v * 8388607.0f);
        fputc(s & 255, f); fputc((s >> 8) & 255, f); fputc((s >> 16) & 255, f);
    }
    fclose(f);
    return 0;
}

static const char *bar(float v, char *out)
{
    double db = v > 0 ? 20 * log10(v) : -99;
    int n = (int)((db + 60) / 3);
    if (n < 0) n = 0;
    if (n > 20) n = 20;
    int k = 0;
    for (int i = 0; i < 20; i++) {
        const char *s = i < n ? "\xe2\x96\x88" : " ";
        size_t l = strlen(s);
        memcpy(out + k, s, l);
        k += (int)l;
    }
    sprintf(out + k, " %6.1f dB", db);
    return out;
}

int main(int argc, char **argv)
{
    const char *cmd = argc > 1 ? argv[1] : "info";
    int rate = argc > 2 ? atoi(argv[2]) : 48000;
    double secs = argc > 3 ? atof(argv[3]) : 5.0;
    const char *wav = argc > 4 ? argv[4] : "test_tascam.wav";

    ctx_t c;
    memset(&c, 0, sizeof c);
    c.rate = rate;
    c.bipfreq = getenv("TASCAM_BIPFREQ") ? atof(getenv("TASCAM_BIPFREQ")) : 440.0;
    if (!strcmp(cmd, "bip")) c.mode = 1;
    else if (!strcmp(cmd, "rec")) c.mode = 2;
    else if (!strcmp(cmd, "thru")) c.mode = 3;
    else if (!strcmp(cmd, "clock")) c.mode = 4;
    else if (strcmp(cmd, "info")) {
        fprintf(stderr, "usage : %s info|bip|rec|thru|clock [fréquence] [secondes] [fichier.wav]\n", argv[0]);
        return 2;
    }

    printf("US-122 MKII présente : %s\n", tascam_device_present() ? "oui" : "non");
    if (c.mode == 0) return 0;

    signal(SIGINT, on_sigint);
    char err[256];
    if (tascam_open(&c.e, err, sizeof err) != 0) {
        fprintf(stderr, "Erreur : %s\n", err);
        return 1;
    }
    if (c.mode == 2) {
        c.rec_cap = (size_t)(secs * rate) + rate;
        c.rec = (float *)calloc(c.rec_cap * 2, sizeof(float));
    }
    tascam_set_input_callback(c.e, on_input, &c);
    tascam_set_output_callback(c.e, on_output, &c);
    c.thru_delay = 4 * tascam_input_chunk_frames(rate);

    if (tascam_start(c.e, rate, err, sizeof err) != 0) {
        fprintf(stderr, "Erreur au démarrage : %s\n", err);
        tascam_close(c.e);
        return 1;
    }
    printf("Flux démarré à %d Hz pendant %.0f s (Ctrl+C pour arrêter)\n", rate, secs);

    double prev_s = -1;
    uint64_t prev_h = 0;
    double tps = tascam_host_ticks_per_second();
    double worst_err_us = 0;
    int n_ts = 0;
    struct timespec t0;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    for (;;) {
        struct timespec ts = { 0, 100000000 };
        nanosleep(&ts, NULL);
        struct timespec t1;
        clock_gettime(CLOCK_MONOTONIC, &t1);
        double el = (t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec) / 1e9;
        if (g_stop || el >= secs) break;

        char a[128], b[128];
        if (c.mode == 4) {
            double s; uint64_t h;
            if (tascam_zero_timestamp(c.e, 4096, &s, &h)) {
                if (prev_s >= 0 && s > prev_s) {
                    double expected = (s - prev_s) / rate;
                    double got = (double)(h - prev_h) / tps;
                    double e_us = fabs(got - expected) * 1e6;
                    if (n_ts > 2 && e_us > worst_err_us) worst_err_us = e_us;
                    n_ts++;
                    printf("horodatage %10.0f  écart %7.1f µs\n", s, (got - expected) * 1e6);
                }
                if (s != prev_s) { prev_s = s; prev_h = h; }
            }
        } else {
            printf("\rG %s | D %s", bar(c.peak[0], a), bar(c.peak[1], b));
            fflush(stdout);
        }
        c.peak[0] = c.peak[1] = 0;
    }

    tascam_stats st;
    tascam_get_stats(c.e, &st);
    tascam_stop(c.e);
    printf("\n\n--- Statistiques ---\n");
    printf("trames reçues      : %llu\n", (unsigned long long)st.in_frames);
    printf("trames envoyées    : %llu\n", (unsigned long long)st.out_frames);
    printf("fréquence mesurée  : %.2f Hz\n", st.measured_rate);
    printf("erreurs entrée     : %llu\n", (unsigned long long)st.in_errors);
    printf("erreurs sortie     : %llu\n", (unsigned long long)st.out_errors);
    printf("sorties sautées    : %llu\n", (unsigned long long)st.out_skipped);
    printf("paquets courts     : %llu\n", (unsigned long long)st.short_packets);
    printf("trous dans le flux : %llu\n", (unsigned long long)st.resyncs);
    printf("paquets vides      : %llu\n", (unsigned long long)st.zero_packets);
    printf("paquets en erreur  : %llu\n", (unsigned long long)st.pkt_errors);
    printf("retards sortie     : %llu\n", (unsigned long long)st.out_gaps);
    printf("ajustements file   : %u\n", st.fifo_depth);
    printf("recalages sortie   : %llu (dérive %lld)\n", (unsigned long long)st.realigns, (long long)st.lead_drift);
    if (c.mode == 4) printf("pire écart horloge : %.1f µs\n", worst_err_us);
    printf("latence sortie env.: %.1f ms\n", 1000.0 * tascam_output_lead_frames(rate) / rate);
    if (c.mode == 3) printf("retard du retour   : %.1f ms (+ latence sortie)\n", 1000.0 * c.thru_delay / rate);

    if (c.mode == 2) {
        if (write_wav(wav, c.rec, c.rec_len, rate) == 0)
            printf("WAV écrit : %s (%.1f s)\n", wav, (double)c.rec_len / rate);
        else
            fprintf(stderr, "impossible d'écrire %s\n", wav);
        free(c.rec);
    }
    tascam_close(c.e);
    return 0;
}
