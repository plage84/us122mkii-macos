/*
 * tascam_usb.c — moteur USB pour la TASCAM US-122 MKII
 * Plage 84 — driver maison pour Apple Silicon
 *
 * Fonctionnement :
 *   - un thread "événements USB" (temps réel) traite toutes les fins de
 *     transferts ;
 *   - chaque transfert = TP_PKTS micro-trames (125 µs chacune) ;
 *   - à chaque bloc reçu en entrée, on envoie en sortie un bloc avec
 *     EXACTEMENT les mêmes tailles de paquets (retour implicite) : la
 *     sortie suit ainsi l'horloge de la carte, sans dérive ni clic ;
 *   - deux anneaux float32 stéréo font le lien avec le client
 *     (outil de test ou plugin Core Audio).
 */
#include "tascam_usb.h"

#include <libusb.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#ifdef __APPLE__
#include <mach/mach.h>
#include <mach/mach_time.h>
#include <mach/thread_policy.h>
#endif

/* ---------------------------------------------------------------- réglages */
#define TP_PKTS          16     /* paquets par transfert = 2 ms            */
#define TP_IN_XFERS      16     /* transferts entrée en vol (32 ms)        */
#define TP_OUT_POOL      16     /* transferts sortie alloués (max)         */
#define TP_OUT_DEFAULT   16     /* transferts sortie en vol (32 ms) : en dessous, macOS laisse des trous et la carte décroche */
#define TP_SIZE_FIFO     4096   /* tailles de paquets reçus (puiss. de 2)  */
#define TP_SIZE_DELAY    32     /* retard sortie/entrée en paquets (4 ms)  */
#define TP_MAX_PKT       78     /* wMaxPacketSize des endpoints iso        */
#define TP_BPF           6      /* octets par trame : 2 canaux x 24 bits   */
#define TP_MAX_FPP       (TP_MAX_PKT / TP_BPF)   /* 13 trames max/paquet   */
#define TP_RING          32768  /* trames (puissance de 2)                 */
#define TP_RING_MASK     (TP_RING - 1)
#define TP_EP_IN         0x81
#define TP_EP_OUT        0x02
#define TP_IFACE_CTRL    0
#define TP_IFACE_AUDIO   1
#define TP_PKT_SECONDS   0.000125

/* ------------------------------------------------------------ journal */
static tascam_log_fn g_log = NULL;
void tascam_set_logger(tascam_log_fn fn) { g_log = fn; }

static void tlog(const char *fmt, ...)
{
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (g_log) g_log(buf);
    else fprintf(stderr, "[tascam] %s\n", buf);
}

/* ------------------------------------------------------------ horloge hôte */
#ifdef __APPLE__
static double g_ticks_per_sec = 0;
uint64_t tascam_host_now(void) { return mach_absolute_time(); }
double tascam_host_ticks_per_second(void)
{
    if (g_ticks_per_sec == 0) {
        mach_timebase_info_data_t tb;
        mach_timebase_info(&tb);
        g_ticks_per_sec = 1e9 * (double)tb.denom / (double)tb.numer;
    }
    return g_ticks_per_sec;
}
#else
uint64_t tascam_host_now(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}
double tascam_host_ticks_per_second(void) { return 1e9; }
#endif

/* ------------------------------------------------------------ moteur */
struct tascam_engine {
    libusb_context       *ctx;
    libusb_device_handle *h;

    pthread_t   thread;
    atomic_int  thread_run;

    int         rate;
    atomic_int  running;
    atomic_int  stopping;
    atomic_int  lost;
    atomic_int  pending;          /* transferts soumis non terminés */
    atomic_int  stuck;            /* transferts jamais revenus (veille) : ne pas libérer */
    atomic_int  got_data;         /* première trame reçue */

    struct libusb_transfer *in_x[TP_IN_XFERS];
    struct libusb_transfer *out_x[TP_OUT_POOL];
    uint8_t      sz_fifo[TP_SIZE_FIFO];   /* tailles reçues, rejouées en sortie */
    uint32_t     sz_w, sz_r;
    uint32_t     sz_debt;      /* paquets envoyés au nominal faute de mesure */
    uint32_t     late_cnt;     /* rappels USB en retard consécutifs */
    unsigned char in_buf[TP_IN_XFERS][TP_PKTS * TP_MAX_PKT];
    unsigned char out_buf[TP_OUT_POOL][TP_PKTS * TP_MAX_PKT];
    pthread_mutex_t out_lock;     /* protège le remplissage de la sortie */

    /* anneaux audio, float32 entrelacé stéréo */
    float       *in_ring;
    float       *out_ring;
    _Atomic uint64_t in_pos;      /* trames capturées (temps échantillon) */
    _Atomic uint64_t out_pos;     /* prochaine trame de sortie à lire */

    /* débit nominal pour l'amorçage */
    double      nominal_fpp;
    double      nominal_acc;

    /* estimation d'horloge (écrite par le thread USB, lue via seqlock) */
    atomic_uint clk_seq;
    double      clk_t0;           /* heure hôte (ticks) du paquet 0 */
    double      clk_fpp;          /* trames par paquet mesurées */
    uint64_t    clk_frames;       /* ancre : trames */
    uint64_t    clk_packets;      /* ancre : paquets */
    int         clk_valid;
    uint64_t    packets;          /* paquets depuis la première donnée */
    double      tpp;              /* ticks hôte par paquet */

    /* statistiques */
    _Atomic uint64_t st_out_frames, st_in_err, st_out_err, st_out_skip,
                     st_short, st_late;

    uint64_t    st_resync;
    int64_t     lead_target;     /* out_pos - in_pos visé */
    int         lead_set;
    _Atomic int64_t lead_now;
    _Atomic uint64_t st_realign;
    _Atomic uint64_t st_adjust;
    _Atomic uint64_t st_zero;
    _Atomic uint64_t st_pkt_err;
    int         packsize;        /* octets max par paquet d'entrée */
    int         outq;            /* transferts sortie en vol */
    uint64_t    last_out_done;   /* heure du dernier transfert sortie terminé */
    _Atomic uint64_t st_out_gap; /* retards de la sortie (> 3 ms) */
    _Atomic uint64_t st_late_max; /* pire manque lors d'une lecture tardive */
    _Atomic uint64_t st_anomaly;  /* fenêtres d'entrée à moitié nulles */
    uint32_t    mon_zero, mon_frames;
    int64_t     carry;           /* trames envoyées en trop (+) ou en moins (-) */
    double      lead_ema;
    tascam_input_cb in_cb;  void *in_cb_ctx;
    tascam_output_cb out_cb; void *out_cb_ctx;
    tascam_lost_cb  lost_cb; void *lost_cb_ctx;
};

/* ---------------------------------------------------- conversions 24 bits */
static inline float s24_to_f(const unsigned char *p)
{
    int32_t v = (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                          ((uint32_t)p[2] << 16));
    v = (v << 8) >> 8;                       /* extension de signe */
    return (float)v * (1.0f / 8388608.0f);
}

static inline void f_to_s24(float f, unsigned char *p)
{
    if (f > 1.0f) f = 1.0f;
    if (f < -1.0f) f = -1.0f;
    int32_t v = (int32_t)lrintf(f * 8388607.0f);
    p[0] = (unsigned char)(v & 0xff);
    p[1] = (unsigned char)((v >> 8) & 0xff);
    p[2] = (unsigned char)((v >> 16) & 0xff);
}

/* ---------------------------------------------------- thread temps réel */
static void make_realtime(void)
{
#ifdef __APPLE__
    double tps = tascam_host_ticks_per_second();
    thread_time_constraint_policy_data_t p;
    p.period      = (uint32_t)(tps * 0.001);    /* 1 ms   */
    p.computation = (uint32_t)(tps * 0.0002);   /* 0,2 ms */
    p.constraint  = (uint32_t)(tps * 0.0008);   /* 0,8 ms */
    p.preemptible = 1;
    kern_return_t kr = thread_policy_set(pthread_mach_thread_np(pthread_self()),
                                         THREAD_TIME_CONSTRAINT_POLICY,
                                         (thread_policy_t)&p,
                                         THREAD_TIME_CONSTRAINT_POLICY_COUNT);
    if (kr != KERN_SUCCESS) tlog("priorité temps réel refusée (%d)", kr);
#endif
}

static void *event_thread(void *arg)
{
    tascam_engine *e = (tascam_engine *)arg;
    make_realtime();
    while (atomic_load(&e->thread_run)) {
        struct timeval tv = { 0, 100000 };
        int r = libusb_handle_events_timeout_completed(e->ctx, &tv, NULL);
        if (r != 0 && r != LIBUSB_ERROR_INTERRUPTED && r != LIBUSB_ERROR_TIMEOUT) {
            tlog("handle_events : %s", libusb_error_name(r));
            struct timespec ts = { 0, 10000000 };
            nanosleep(&ts, NULL);
        }
    }
    return NULL;
}

/* ------------------------------------------------------------ utilitaires */
static void mark_lost(tascam_engine *e)
{
    if (atomic_exchange(&e->lost, 1) == 0) {
        tlog("carte débranchée");
        if (e->lost_cb) e->lost_cb(e->lost_cb_ctx);
    }
}

static int nominal_frames_next(tascam_engine *e)
{
    e->nominal_acc += e->nominal_fpp;
    int n = (int)e->nominal_acc;
    e->nominal_acc -= n;
    if (n > TP_MAX_FPP) n = TP_MAX_FPP;
    return n;
}

/*
 * Remplit et soumet un transfert de sortie. La sortie tourne en continu,
 * indépendamment de l'entrée ; chaque paquet reprend la taille d'un paquet
 * reçu 4 ms plus tôt, donc la carte reçoit exactement son propre débit.
 */
static void LIBUSB_CALL out_done(struct libusb_transfer *t);

static void fill_and_submit_output(tascam_engine *e, struct libusb_transfer *t)
{
    pthread_mutex_lock(&e->out_lock);
    int have_data = atomic_load_explicit(&e->got_data, memory_order_acquire);
    uint64_t pos = atomic_load_explicit(&e->out_pos, memory_order_relaxed);
    uint8_t sizes[TP_PKTS];
    uint32_t total = 0;
    float tmp[TP_PKTS * TP_MAX_FPP * 2];
    if (have_data) {
        /* la sortie doit rester calée sur l'entrée (même horloge) :
           si l'écart dérive, on se recale */
        int64_t in_now = (int64_t)atomic_load_explicit(&e->in_pos, memory_order_acquire);
        int64_t lead = (int64_t)pos - in_now;
        if (!e->lead_set) {
            /* on lit un peu derrière l'entrée : macOS a toujours écrit
               ces trames depuis longtemps (pas de lecture de zones vides) */
            int64_t np = in_now - (int64_t)tascam_input_chunk_frames(e->rate);
            if (np < 0) np = 0;
            pos = (uint64_t)np;
            lead = np - in_now;
            e->lead_target = lead;
            e->lead_set = 1;
            e->lead_ema = 0;
        } else if (llabs(lead - e->lead_target) > 4 * TP_PKTS * TP_MAX_FPP) {
            int64_t np = in_now + e->lead_target;
            if (np < 0) np = 0;
            pos = (uint64_t)np;
            if (atomic_fetch_add(&e->st_realign, 1) < 5)
                tlog("sortie recalée (écart %lld trames)", (long long)(lead - e->lead_target));
            lead = e->lead_target;
        }
        atomic_store_explicit(&e->lead_now, lead - e->lead_target, memory_order_relaxed);
    }

    /* tailles des paquets : on rejoue les tailles reçues en entrée avec un
       retard fixe de TP_SIZE_DELAY paquets (retour implicite, comme le pilote
       Linux) ; la carte reçoit ainsi exactement ce qu'elle consomme. */
    if (have_data) {
        uint32_t depth = e->sz_w - e->sz_r;
        if (depth > TP_SIZE_DELAY + 24) {          /* trop d'avance : on rattrape */
            uint32_t drop = depth - TP_SIZE_DELAY;
            for (uint32_t k = 0; k < drop; k++)       /* trames sautées : à rajouter */
                e->sz_r++;   /* paquets que la sortie a ratés : on ne les rattrape PAS
                                (envoyer des paquets plus gros dérègle la carte) */
            atomic_fetch_add(&e->st_adjust, 1);
        }
    }
    for (int i = 0; i < TP_PKTS; i++) {
        int n;
        if (!have_data) {
            n = nominal_frames_next(e);
        } else {
            if (e->sz_r != e->sz_w) {
                n = e->sz_fifo[e->sz_r++ & (TP_SIZE_FIFO - 1)];
            } else {                                      /* file vide */
                n = nominal_frames_next(e);
                e->carry += n;                            /* à rendre plus tard */
                atomic_fetch_add(&e->st_adjust, 1);
            }
            /* macOS refuse les paquets vides : on envoie 1 trame et on la
               rendra sur les paquets suivants (total exact) */
            if (n == 0) { n = 1; e->carry += 1; }
            else if (e->carry > 0 && n > 1) { n -= 1; e->carry -= 1; }
            else if (e->carry < 0 && n < TP_MAX_FPP) { n += 1; e->carry += 1; }
        }
        if (n > (e->packsize ? e->packsize : TP_MAX_PKT) / TP_BPF) n = (e->packsize ? e->packsize : TP_MAX_PKT) / TP_BPF;
        sizes[i] = (uint8_t)n;
        total += (uint32_t)n;
    }

    if (have_data) {
        for (uint32_t f = 0; f < total; f++) {
            uint32_t idx = (uint32_t)((pos + f) & TP_RING_MASK) * 2;
            tmp[f * 2] = e->out_ring[idx];
            tmp[f * 2 + 1] = e->out_ring[idx + 1];
            e->out_ring[idx] = 0.0f;          /* évite de rejouer */
            e->out_ring[idx + 1] = 0.0f;
        }
        if (e->out_cb) e->out_cb(e->out_cb_ctx, tmp, total, pos);
        atomic_store_explicit(&e->out_pos, pos + total, memory_order_release);
    } else {
        memset(tmp, 0, total * 2 * sizeof(float));
    }

    unsigned char *p = t->buffer;
    for (int i = 0; i < TP_PKTS; i++) t->iso_packet_desc[i].length = sizes[i] * TP_BPF;
    for (uint32_t f = 0; f < total; f++) {
        f_to_s24(tmp[f * 2], p);
        f_to_s24(tmp[f * 2 + 1], p + 3);
        p += TP_BPF;
    }
    t->length = (int)(total * TP_BPF);
    pthread_mutex_unlock(&e->out_lock);

    atomic_fetch_add(&e->pending, 1);
    int r = libusb_submit_transfer(t);
    if (r != 0) {
        atomic_fetch_sub(&e->pending, 1);
        atomic_fetch_add(&e->st_out_err, 1);
        if (r == LIBUSB_ERROR_NO_DEVICE) mark_lost(e);
        else {
            char b[160]; int k = 0;
            for (int i = 0; i < TP_PKTS && k < 150; i++)
                k += snprintf(b + k, sizeof b - k, "%u ", t->iso_packet_desc[i].length);
            tlog("soumission sortie : %s (longueur %d, paquets : %s)",
                 libusb_error_name(r), t->length, b);
        }
    } else {
        atomic_fetch_add(&e->st_out_frames, total);
    }
}

static void LIBUSB_CALL out_done(struct libusb_transfer *t)
{
    tascam_engine *e = (tascam_engine *)t->user_data;
    atomic_fetch_sub(&e->pending, 1);
    if (t->status == LIBUSB_TRANSFER_NO_DEVICE) { mark_lost(e); return; }
    if (atomic_load(&e->stopping) || t->status == LIBUSB_TRANSFER_CANCELLED) return;
    if (t->status != LIBUSB_TRANSFER_COMPLETED) atomic_fetch_add(&e->st_out_err, 1);
    uint64_t now = tascam_host_now();
    if (e->last_out_done &&
        (double)(now - e->last_out_done) > 0.003 * tascam_host_ticks_per_second())
        atomic_fetch_add(&e->st_out_gap, 1);
    e->last_out_done = now;
    fill_and_submit_output(e, t);
}

/* Met à jour l'estimation d'horloge après un bloc reçu. */
static void clock_update(tascam_engine *e, uint64_t now)
{
    uint64_t F = atomic_load_explicit(&e->in_pos, memory_order_relaxed);
    uint64_t P = e->packets;
    double cand = (double)now - (double)P * e->tpp;

    double t0 = e->clk_t0, fpp = e->clk_fpp;
    if (!e->clk_valid) {
        t0 = cand;
        fpp = e->nominal_fpp;
    } else {
        /* minimum glissant : la latence la plus faible = vraie heure bus */
        if (cand < t0) t0 = cand;
        else if (cand - t0 > 0.004 * tascam_host_ticks_per_second()) {
            /* rappel en retard : en général le thread a juste été retardé
               (les données ne sont pas perdues). On ne se recale que si le
               décalage persiste 100 ms (vrai trou dans le flux). */
            if (++e->late_cnt >= 50) {
                t0 = cand;
                e->late_cnt = 0;
                if (++e->st_resync <= 5) tlog("horloge recalée (trou dans le flux USB)");
            }
        }
        else {
            e->late_cnt = 0;
            t0 += (cand - t0) * 0.0005;
        }
        if (P > 4000) fpp = (double)F / (double)P;   /* > 0,5 s de mesure */
    }

    atomic_fetch_add_explicit(&e->clk_seq, 1, memory_order_acq_rel);  /* impair */
    e->clk_t0 = t0;
    e->clk_fpp = fpp;
    e->clk_frames = F;
    e->clk_packets = P;
    e->clk_valid = 1;
    atomic_fetch_add_explicit(&e->clk_seq, 1, memory_order_acq_rel);  /* pair */
}

static void LIBUSB_CALL in_done(struct libusb_transfer *t)
{
    tascam_engine *e = (tascam_engine *)t->user_data;
    uint64_t now = tascam_host_now();

    if (t->status == LIBUSB_TRANSFER_NO_DEVICE) {
        mark_lost(e);
        atomic_fetch_sub(&e->pending, 1);
        return;
    }
    if (atomic_load(&e->stopping) || t->status == LIBUSB_TRANSFER_CANCELLED) {
        atomic_fetch_sub(&e->pending, 1);
        return;
    }

    uint8_t sizes[TP_PKTS];
    uint32_t total = 0;
    float tmp[TP_PKTS * TP_MAX_FPP * 2];

    if (t->status == LIBUSB_TRANSFER_COMPLETED) {
        for (int i = 0; i < TP_PKTS; i++) {
            struct libusb_iso_packet_descriptor *d = &t->iso_packet_desc[i];
            int frames = 0, missing = 0;
            if (d->status == LIBUSB_TRANSFER_COMPLETED)
                frames = (int)d->actual_length / TP_BPF;
            else {
                if (atomic_fetch_add(&e->st_pkt_err, 1) < 5)
                    tlog("paquet d'entrée en erreur (statut %d, longueur %u)", (int)d->status, d->actual_length);
            }
            if (frames > (e->packsize ? e->packsize : TP_MAX_PKT) / TP_BPF) frames = (e->packsize ? e->packsize : TP_MAX_PKT) / TP_BPF;
            /* paquet perdu ou vide une fois le flux lancé : la carte a
               quand même produit ~1 paquet nominal ; on bouche le trou
               avec du silence pour garder l'horloge et la sortie alignées */
            if (frames == 0 && atomic_load_explicit(&e->got_data, memory_order_relaxed)) {
                frames = (int)(e->nominal_fpp + 0.5);
                missing = 1;
                atomic_fetch_add(&e->st_zero, 1);
            }
            const unsigned char *p = libusb_get_iso_packet_buffer_simple(t, (unsigned)i);
            for (int f = 0; f < frames; f++) {
                if (missing) {
                    tmp[(total + f) * 2] = 0.0f;
                    tmp[(total + f) * 2 + 1] = 0.0f;
                } else {
                    tmp[(total + f) * 2]     = s24_to_f(p + f * TP_BPF);
                    tmp[(total + f) * 2 + 1] = s24_to_f(p + f * TP_BPF + 3);
                }
            }
            sizes[i] = (uint8_t)frames;
            total += (uint32_t)frames;
            if (frames > 0 && frames < (int)e->nominal_fpp)
                atomic_fetch_add(&e->st_short, 1);
        }
    } else {
        atomic_fetch_add(&e->st_in_err, 1);
        if (atomic_fetch_add(&e->st_pkt_err, 1) < 5)
            tlog("transfert d'entrée en erreur (statut %d)", (int)t->status);
        memset(sizes, 0, sizeof sizes);
        if (atomic_load_explicit(&e->got_data, memory_order_relaxed)) {
            int fr = (int)(e->nominal_fpp + 0.5);
            for (int i = 0; i < TP_PKTS; i++) {
                for (int f = 0; f < fr; f++) { tmp[(total + f) * 2] = 0; tmp[(total + f) * 2 + 1] = 0; }
                sizes[i] = (uint8_t)fr;
                total += (uint32_t)fr;
                atomic_fetch_add(&e->st_zero, 1);
            }
        }
    }

    int had_data = atomic_load_explicit(&e->got_data, memory_order_relaxed);
    if (!had_data && total > 0) {
        /* première vraie donnée : le temps échantillon démarre ici */
        atomic_store_explicit(&e->in_pos, 0, memory_order_relaxed);
        atomic_store_explicit(&e->out_pos, 0, memory_order_relaxed);
        e->packets = 0;
        pthread_mutex_lock(&e->out_lock);
        e->sz_w = e->sz_r = 0;
        e->sz_debt = 0;
        e->late_cnt = 0;
        e->carry = 0;
        atomic_store(&e->st_zero, 0);
        atomic_store(&e->st_pkt_err, 0);
        atomic_store(&e->st_out_gap, 0);
        atomic_store(&e->st_late_max, 0);
        atomic_store(&e->st_anomaly, 0);
        e->mon_zero = e->mon_frames = 0;
        for (int i = 0; i < TP_SIZE_DELAY; i++)
            e->sz_fifo[e->sz_w++ & (TP_SIZE_FIFO - 1)] = (uint8_t)nominal_frames_next(e);
        e->lead_set = 0;
        pthread_mutex_unlock(&e->out_lock);
        atomic_store_explicit(&e->got_data, 1, memory_order_release);
        had_data = 1;
        tlog("flux audio démarré (%d Hz)", e->rate);
    }

    if (had_data) {
        uint64_t pos = atomic_load_explicit(&e->in_pos, memory_order_relaxed);
        for (uint32_t f = 0; f < total; f++) {
            uint32_t idx = (uint32_t)((pos + f) & TP_RING_MASK) * 2;
            e->in_ring[idx]     = tmp[f * 2];
            e->in_ring[idx + 1] = tmp[f * 2 + 1];
        }
        atomic_store_explicit(&e->in_pos, pos + total, memory_order_release);

        /* surveillance : la carte peut passer dans un état où elle envoie
           3 trames sur 6 à zéro (son "réduit en bits"). On le détecte. */
        for (uint32_t f = 0; f < total; f++)
            if (tmp[f * 2] == 0.0f && tmp[f * 2 + 1] == 0.0f) e->mon_zero++;
        e->mon_frames += total;
        if (e->mon_frames >= 12000) {           /* ~0,25 s */
            double zf = (double)e->mon_zero / (double)e->mon_frames;
            if (zf > 0.3 && zf < 0.9) {
                if (atomic_fetch_add(&e->st_anomaly, 1) < 5)
                    tlog("ENTRÉE ANORMALE : %.0f %% de trames nulles (position %llu)",
                         zf * 100.0, (unsigned long long)pos);
            }
            e->mon_zero = e->mon_frames = 0;
        }
        e->packets += TP_PKTS;
        clock_update(e, now);
        if (e->in_cb && total) e->in_cb(e->in_cb_ctx, tmp, total, pos);
    }

    /* tailles reçues -> file rejouée par la sortie */
    if (had_data && total > 0) {
        pthread_mutex_lock(&e->out_lock);
        for (int i = 0; i < TP_PKTS; i++) {
            if (e->sz_w - e->sz_r < TP_SIZE_FIFO) e->sz_fifo[e->sz_w++ & (TP_SIZE_FIFO - 1)] = sizes[i];
        }
        pthread_mutex_unlock(&e->out_lock);
    }

    /* on relance la capture */
    int r = libusb_submit_transfer(t);
    if (r != 0) {
        atomic_fetch_sub(&e->pending, 1);
        atomic_fetch_add(&e->st_in_err, 1);
        if (r == LIBUSB_ERROR_NO_DEVICE) mark_lost(e);
        else tlog("resoumission entrée : %s", libusb_error_name(r));
    }
}

/* ------------------------------------------------------------ présence */
static pthread_mutex_t g_probe_lock = PTHREAD_MUTEX_INITIALIZER;
static libusb_context *g_probe_ctx = NULL;

int tascam_device_present(void)
{
    int found = 0;
    pthread_mutex_lock(&g_probe_lock);
    if (!g_probe_ctx && libusb_init(&g_probe_ctx) != 0) g_probe_ctx = NULL;
    if (g_probe_ctx) {
        libusb_device **list;
        ssize_t n = libusb_get_device_list(g_probe_ctx, &list);
        for (ssize_t i = 0; i < n; i++) {
            struct libusb_device_descriptor d;
            if (libusb_get_device_descriptor(list[i], &d) == 0 &&
                d.idVendor == TASCAM_VID && d.idProduct == TASCAM_PID) {
                found = 1;
                break;
            }
        }
        if (n >= 0) libusb_free_device_list(list, 1);
    }
    pthread_mutex_unlock(&g_probe_lock);
    return found;
}

/* ------------------------------------------------------------ ouverture */
int tascam_open(tascam_engine **out, char *err, size_t errlen)
{
    *out = NULL;
    tascam_engine *e = (tascam_engine *)calloc(1, sizeof *e);
    if (!e) { snprintf(err, errlen, "mémoire insuffisante"); return -1; }
    e->in_ring = (float *)calloc(TP_RING * 2, sizeof(float));
    e->out_ring = (float *)calloc(TP_RING * 2, sizeof(float));
    pthread_mutex_init(&e->out_lock, NULL);
    e->tpp = TP_PKT_SECONDS * tascam_host_ticks_per_second();

    int r = libusb_init(&e->ctx);
    if (r) { snprintf(err, errlen, "libusb_init : %s", libusb_error_name(r)); goto fail; }

    e->h = libusb_open_device_with_vid_pid(e->ctx, TASCAM_VID, TASCAM_PID);
    if (!e->h) { snprintf(err, errlen, "US-122 MKII introuvable (branchée ?)"); goto fail; }

    for (int i = TP_IFACE_CTRL; i <= TP_IFACE_AUDIO; i++) {
        r = libusb_claim_interface(e->h, i);
        if (r) {
            snprintf(err, errlen, "impossible de prendre l'interface %d : %s "
                     "(un autre programme utilise-t-il la carte ?)", i, libusb_error_name(r));
            goto fail;
        }
    }

    for (int i = 0; i < TP_IN_XFERS; i++) {
        struct libusb_transfer *t = libusb_alloc_transfer(TP_PKTS);
        libusb_fill_iso_transfer(t, e->h, TP_EP_IN, e->in_buf[i],
                                 TP_PKTS * TP_MAX_PKT, TP_PKTS, in_done, e, 1000);
        libusb_set_iso_packet_lengths(t, TP_MAX_PKT);
        e->in_x[i] = t;
    }
    for (int i = 0; i < TP_OUT_POOL; i++) {
        struct libusb_transfer *t = libusb_alloc_transfer(TP_PKTS);
        libusb_fill_iso_transfer(t, e->h, TP_EP_OUT, e->out_buf[i],
                                 TP_PKTS * TP_MAX_PKT, TP_PKTS, out_done, e, 1000);
        e->out_x[i] = t;
    }

    atomic_store(&e->thread_run, 1);
    if (pthread_create(&e->thread, NULL, event_thread, e) != 0) {
        atomic_store(&e->thread_run, 0);
        snprintf(err, errlen, "impossible de créer le thread USB");
        goto fail;
    }
    *out = e;
    return 0;

fail:
    tascam_close(e);
    return -1;
}

void tascam_close(tascam_engine *e)
{
    if (!e) return;
    if (atomic_load(&e->running)) tascam_stop(e);
    if (atomic_exchange(&e->thread_run, 0)) {
        if (e->h) libusb_interrupt_event_handler(e->ctx);
        pthread_join(e->thread, NULL);
    }
    if (atomic_load(&e->stuck)) {
        /* des transferts n'ont jamais été rendus par macOS (sortie de veille) :
           les libérer ferait planter Core Audio. On abandonne ces quelques Ko. */
        tlog("fermeture : ressources USB abandonnées (transferts bloqués)");
        return;
    }
    for (int i = 0; i < TP_IN_XFERS; i++) if (e->in_x[i]) libusb_free_transfer(e->in_x[i]);
    for (int i = 0; i < TP_OUT_POOL; i++) if (e->out_x[i]) libusb_free_transfer(e->out_x[i]);
    if (e->h) {
        libusb_release_interface(e->h, TP_IFACE_AUDIO);
        libusb_release_interface(e->h, TP_IFACE_CTRL);
        libusb_close(e->h);
    }
    if (e->ctx) libusb_exit(e->ctx);
    free(e->in_ring);
    free(e->out_ring);
    pthread_mutex_destroy(&e->out_lock);
    free(e);
}

/* ------------------------------------------------------------ démarrage */
static int set_rate(tascam_engine *e, int rate)
{
    unsigned char d[3] = { (unsigned char)(rate & 0xff),
                           (unsigned char)((rate >> 8) & 0xff),
                           (unsigned char)((rate >> 16) & 0xff) };
    /* le pilote Linux dédié (us122l) ne règle que l'endpoint d'entrée ;
       TASCAM_RATE_BOTH=1 pour aussi régler la sortie (ancien comportement) */
    int eps[2] = { TP_EP_IN, TP_EP_OUT };
    int n_eps = getenv("TASCAM_RATE_BOTH") ? 2 : 1;
    for (int i = 0; i < n_eps; i++) {
        int r = libusb_control_transfer(e->h, 0x22, 0x01, 0x0100,
                                        (uint16_t)eps[i], d, 3, 1000);
        if (r < 0) {
            tlog("réglage %d Hz refusé sur 0x%02x : %s", rate, eps[i], libusb_error_name(r));
            return r;
        }
    }
    return 0;
}

int tascam_start(tascam_engine *e, int rate, char *err, size_t errlen)
{
    if (atomic_load(&e->running)) {
        if (e->rate == rate) return 0;
        tascam_stop(e);
    }
    if (rate != 44100 && rate != 48000 && rate != 88200 && rate != 96000) {
        snprintf(err, errlen, "fréquence non gérée : %d", rate);
        return -1;
    }
    if (atomic_load(&e->lost)) { snprintf(err, errlen, "carte débranchée"); return -1; }

    /* initialisation propre à Tascam, reprise du pilote Linux us122l :
       requête constructeur 'I' avec 0x11 puis 0x10 (TASCAM_NOINIT=1 pour la sauter) */
    if (!getenv("TASCAM_NOINIT")) {
        int a = libusb_control_transfer(e->h, 0x40, 'I', 0x11, 0, NULL, 0, 1000);
        int b = libusb_control_transfer(e->h, 0x40, 'I', 0x10, 0, NULL, 0, 1000);
        tlog("init Tascam : %s / %s", a < 0 ? libusb_error_name(a) : "ok",
             b < 0 ? libusb_error_name(b) : "ok");
    }

    /* taille des paquets d'entrée, comme Linux : 36/42/72 octets à
       44,1/48/88,2 kHz, 78 à 96 kHz (TASCAM_PACK=78 pour forcer le maximum) */
    e->packsize = rate == 44100 ? 36 : rate == 48000 ? 42 : rate == 88200 ? 72 : TP_MAX_PKT;
    if (getenv("TASCAM_PACK")) {
        int v = atoi(getenv("TASCAM_PACK"));
        if (v >= 6 && v <= TP_MAX_PKT && v % TP_BPF == 0) e->packsize = v;
    }
    tlog("paquets d'entrée de %d octets", e->packsize);

    int r = libusb_set_interface_alt_setting(e->h, TP_IFACE_AUDIO, 1);
    if (r) { snprintf(err, errlen, "altsetting 1 : %s", libusb_error_name(r)); return -1; }
    if (set_rate(e, rate) < 0) { snprintf(err, errlen, "réglage de fréquence refusé"); return -1; }

    e->rate = rate;
    e->nominal_fpp = rate / 8000.0;
    e->nominal_acc = 0;
    e->packets = 0;
    e->clk_valid = 0;
    e->st_resync = 0;
    e->lead_set = 0;
    atomic_store(&e->st_realign, 0);
    atomic_store(&e->st_adjust, 0);
    atomic_store(&e->in_pos, 0);
    atomic_store(&e->out_pos, 0);
    atomic_store(&e->got_data, 0);
    atomic_store(&e->stopping, 0);
    memset(e->in_ring, 0, TP_RING * 2 * sizeof(float));
    memset(e->out_ring, 0, TP_RING * 2 * sizeof(float));
    e->sz_w = e->sz_r = 0;
    e->sz_debt = 0;
    e->late_cnt = 0;
    atomic_store(&e->running, 1);

    /* amorçage : sortie au débit nominal, puis capture */
    e->outq = TP_OUT_DEFAULT;
    if (getenv("TASCAM_OUTQ")) {
        int v = atoi(getenv("TASCAM_OUTQ"));
        if (v >= 2 && v <= TP_OUT_POOL) e->outq = v;
    }
    tlog("sortie : %d transferts en vol (%d ms)", e->outq, e->outq * TP_PKTS / 8);
    e->last_out_done = 0;
    for (int k = 0; k < e->outq; k++) fill_and_submit_output(e, e->out_x[k]);
    for (int i = 0; i < TP_IN_XFERS; i++) {
        e->in_x[i]->length = TP_PKTS * e->packsize;
        libusb_set_iso_packet_lengths(e->in_x[i], (unsigned)e->packsize);
        atomic_fetch_add(&e->pending, 1);
        r = libusb_submit_transfer(e->in_x[i]);
        if (r) {
            atomic_fetch_sub(&e->pending, 1);
            snprintf(err, errlen, "soumission entrée : %s", libusb_error_name(r));
            tascam_stop(e);
            return -1;
        }
    }

    /* on attend la première vraie donnée (jusqu'à 1,5 s) */
    for (int i = 0; i < 150 && !atomic_load(&e->got_data); i++) {
        struct timespec ts = { 0, 10000000 };
        nanosleep(&ts, NULL);
    }
    if (!atomic_load(&e->got_data)) {
        snprintf(err, errlen, "la carte n'envoie pas de son (délai dépassé)");
        tascam_stop(e);
        return -1;
    }
    return 0;
}

void tascam_stop(tascam_engine *e)
{
    if (!atomic_load(&e->running)) return;
    atomic_store(&e->stopping, 1);
    for (int i = 0; i < TP_IN_XFERS; i++) libusb_cancel_transfer(e->in_x[i]);
    for (int i = 0; i < TP_OUT_POOL; i++) libusb_cancel_transfer(e->out_x[i]);
    for (int i = 0; i < 200 && atomic_load(&e->pending) > 0; i++) {
        struct timespec ts = { 0, 5000000 };
        nanosleep(&ts, NULL);
    }
    if (atomic_load(&e->pending) > 0) {
        tlog("arrêt : %d transferts encore en cours", atomic_load(&e->pending));
        atomic_store(&e->stuck, 1);
    }
    if (!atomic_load(&e->lost))
        libusb_set_interface_alt_setting(e->h, TP_IFACE_AUDIO, 0);
    atomic_store(&e->pending, 0);
    atomic_store(&e->running, 0);
    atomic_store(&e->got_data, 0);
}

int tascam_is_running(tascam_engine *e) { return e && atomic_load(&e->running); }
int tascam_rate(tascam_engine *e) { return e ? e->rate : 0; }

uint32_t tascam_input_chunk_frames(int rate)
{
    return (uint32_t)ceil(TP_PKTS * (rate / 8000.0)) + 1;
}

uint32_t tascam_output_lead_frames(int rate)
{
    /* latence de la file de sortie (transferts en vol) */
    return (uint32_t)ceil(TP_OUT_DEFAULT * TP_PKTS * (rate / 8000.0));
}

/* ------------------------------------------------------------ échange */
uint64_t tascam_input_position(tascam_engine *e)
{
    return atomic_load_explicit(&e->in_pos, memory_order_acquire);
}

uint32_t tascam_read_input(tascam_engine *e, uint64_t sample_time,
                           float *dst, uint32_t frames)
{
    uint64_t avail = atomic_load_explicit(&e->in_pos, memory_order_acquire);
    uint32_t ok = 0;
    for (uint32_t i = 0; i < frames; i++) {
        uint64_t t = sample_time + i;
        if (t < avail && avail - t < TP_RING - 1024) {
            uint32_t idx = (uint32_t)(t & TP_RING_MASK) * 2;
            dst[i * 2] = e->in_ring[idx];
            dst[i * 2 + 1] = e->in_ring[idx + 1];
            ok++;
        } else {
            dst[i * 2] = 0.0f;
            dst[i * 2 + 1] = 0.0f;
        }
    }
    if (ok < frames) {
        atomic_fetch_add(&e->st_late, 1);
        /* de combien de trames macOS lit-il trop tôt ? (pour régler la marge) */
        uint64_t miss = sample_time + frames > avail ? sample_time + frames - avail : 0;
        if (miss > atomic_load(&e->st_late_max)) atomic_store(&e->st_late_max, miss);
    }
    return ok;
}

void tascam_write_output(tascam_engine *e, uint64_t sample_time,
                         const float *src, uint32_t frames)
{
    uint64_t rd = atomic_load_explicit(&e->out_pos, memory_order_acquire);
    for (uint32_t i = 0; i < frames; i++) {
        uint64_t t = sample_time + i;
        if (t < rd || t - rd >= TP_RING - 1024) continue;   /* trop tard / trop tôt */
        uint32_t idx = (uint32_t)(t & TP_RING_MASK) * 2;
        e->out_ring[idx] = src[i * 2];
        e->out_ring[idx + 1] = src[i * 2 + 1];
    }
}

/* ------------------------------------------------------------ horloge */
int tascam_zero_timestamp(tascam_engine *e, uint32_t period,
                          double *sample_time, uint64_t *host_time)
{
    double t0, fpp;
    uint64_t F, P;
    int valid;
    unsigned s1, s2;
    int tries = 0;
    do {
        s1 = atomic_load_explicit(&e->clk_seq, memory_order_acquire);
        t0 = e->clk_t0; fpp = e->clk_fpp; F = e->clk_frames; P = e->clk_packets;
        valid = e->clk_valid;
        atomic_thread_fence(memory_order_acquire);
        s2 = atomic_load_explicit(&e->clk_seq, memory_order_acquire);
    } while ((s1 != s2 || (s1 & 1)) && ++tries < 100);

    if (!valid || fpp <= 0) return 0;

    double now = (double)tascam_host_now();
    double pkts_now = (now - t0) / e->tpp;
    double s_now = (double)F + (pkts_now - (double)P) * fpp;
    if (s_now < 0) s_now = 0;
    double zs = floor(s_now / period) * period;
    double host = t0 + ((double)P + (zs - (double)F) / fpp) * e->tpp;
    *sample_time = zs;
    *host_time = (uint64_t)(host > 0 ? host : 0);
    return 1;
}

/* ------------------------------------------------------------ divers */
void tascam_get_stats(tascam_engine *e, tascam_stats *s)
{
    memset(s, 0, sizeof *s);
    s->in_frames = atomic_load(&e->in_pos);
    s->out_frames = atomic_load(&e->st_out_frames);
    s->packets = e->packets;
    s->in_errors = atomic_load(&e->st_in_err);
    s->out_errors = atomic_load(&e->st_out_err);
    s->out_skipped = atomic_load(&e->st_out_skip);
    s->short_packets = atomic_load(&e->st_short);
    s->late_reads = atomic_load(&e->st_late);
    s->resyncs = e->st_resync;
    s->realigns = atomic_load(&e->st_realign);
    s->lead_drift = atomic_load(&e->lead_now);
    s->fifo_depth = (uint32_t)atomic_load(&e->st_adjust);
    s->zero_packets = atomic_load(&e->st_zero);
    s->carry = e->carry;
    s->depth = e->sz_w - e->sz_r;
    s->pkt_errors = atomic_load(&e->st_pkt_err);
    s->out_gaps = atomic_load(&e->st_out_gap);
    s->late_max = atomic_load(&e->st_late_max);
    s->anomalies = atomic_load(&e->st_anomaly);
    s->measured_rate = e->packets ? (double)s->in_frames / (double)e->packets * 8000.0 : 0;
    s->running = atomic_load(&e->running);
    s->device_lost = atomic_load(&e->lost);
}

void tascam_set_input_callback(tascam_engine *e, tascam_input_cb cb, void *ctx)
{
    e->in_cb_ctx = ctx;
    e->in_cb = cb;
}

void tascam_set_lost_callback(tascam_engine *e, tascam_lost_cb cb, void *ctx)
{
    e->lost_cb_ctx = ctx;
    e->lost_cb = cb;
}

void tascam_set_output_callback(tascam_engine *e, tascam_output_cb cb, void *ctx)
{
    e->out_cb_ctx = ctx;
    e->out_cb = cb;
}
