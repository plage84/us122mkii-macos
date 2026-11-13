/*
 * tascam_usb.h — moteur USB pour la TASCAM US-122 MKII (0644:8021)
 * Plage 84 — driver maison pour Apple Silicon
 *
 * Protocole (vérifié sur la carte) :
 *   - interface 1, altsetting 1
 *   - EP 0x81 isochrone IN  : capture, 24 bits LE, 2 canaux
 *   - EP 0x02 isochrone OUT : lecture, même format
 *   - fréquence : SET_CUR (0x22/0x01, wValue 0x0100) sur chaque endpoint
 *   - la capture ne tourne que si la lecture tourne en même temps
 *   - l'horloge est celle de la carte : on renvoie en sortie exactement
 *     le nombre de trames reçues en entrée (retour implicite).
 *
 * Le temps "échantillon" du moteur = nombre total de trames reçues en entrée.
 */
#ifndef TASCAM_USB_H
#define TASCAM_USB_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TASCAM_VID 0x0644
#define TASCAM_PID 0x8021
#define TASCAM_CHANNELS 2

typedef struct tascam_engine tascam_engine;

typedef struct {
    uint64_t in_frames;      /* trames capturées depuis le démarrage */
    uint64_t out_frames;     /* trames envoyées */
    uint64_t packets;        /* micro-trames USB écoulées */
    uint64_t in_errors;      /* transferts entrée en erreur */
    uint64_t out_errors;     /* transferts sortie en erreur */
    uint64_t out_skipped;    /* transferts sortie non soumis (pool vide) */
    uint64_t short_packets;  /* paquets plus courts que la normale */
    uint64_t late_reads;     /* lectures client au-delà des données reçues */
    uint64_t resyncs;        /* trous détectés dans le flux USB */
    uint64_t realigns;       /* recalages sortie/entrée */
    int64_t  lead_drift;     /* dérive actuelle sortie/entrée (trames) */
    uint32_t fifo_depth;     /* ajustements (file vide / trop pleine) */
    uint64_t zero_packets;   /* paquets d'entrée vides envoyés par la carte */
    int64_t  carry;          /* trames à rendre */
    uint32_t depth;          /* profondeur de la file de tailles */
    uint64_t pkt_errors;     /* paquets d'entrée en erreur USB */
    uint64_t out_gaps;       /* sortie servie avec plus de 3 ms de retard */
    uint64_t late_max;       /* pire manque (trames) lors d'une lecture tardive */
    uint64_t anomalies;      /* fenêtres d'entrée à moitié nulles (carte déréglée) */
    double   measured_rate;  /* fréquence mesurée (horloge carte / bus USB) */
    int      running;
    int      device_lost;
} tascam_stats;

/* Horloge hôte (mach_absolute_time sur macOS) */
uint64_t tascam_host_now(void);
double   tascam_host_ticks_per_second(void);

/* Retourne 1 si une US-122 MKII est branchée. */
int  tascam_device_present(void);

/* Ouvre la carte. rate : 44100, 48000, 88200 ou 96000. */
int  tascam_open(tascam_engine **out, char *err, size_t errlen);
void tascam_close(tascam_engine *e);

int  tascam_start(tascam_engine *e, int rate, char *err, size_t errlen);
void tascam_stop(tascam_engine *e);
int  tascam_is_running(tascam_engine *e);
int  tascam_rate(tascam_engine *e);

/* Latence de la file de sortie, en trames. */
uint32_t tascam_output_lead_frames(int rate);
/* Granularité de l'entrée, en trames (pour l'offset de sécurité). */
uint32_t tascam_input_chunk_frames(int rate);

/* Échange audio (float32 entrelacé stéréo), en temps échantillon absolu. */
/* Lit la capture ; retourne le nombre de trames réellement disponibles. */
uint32_t tascam_read_input(tascam_engine *e, uint64_t sample_time,
                           float *dst, uint32_t frames);
/* Écrit (mixe par remplacement) la lecture à jouer à sample_time. */
void tascam_write_output(tascam_engine *e, uint64_t sample_time,
                         const float *src, uint32_t frames);

/* Temps échantillon actuel (trames reçues). */
uint64_t tascam_input_position(tascam_engine *e);

/*
 * Horodatage zéro pour Core Audio : renvoie le dernier multiple de `period`
 * déjà passé et l'heure hôte correspondante, estimée à partir du bus USB
 * (1 paquet = 125 µs). Retourne 0 si pas encore d'estimation fiable.
 */
int tascam_zero_timestamp(tascam_engine *e, uint32_t period,
                          double *sample_time, uint64_t *host_time);

void tascam_get_stats(tascam_engine *e, tascam_stats *s);

/* Callback optionnel appelé (thread USB) après chaque bloc reçu. */
typedef void (*tascam_input_cb)(void *ctx, const float *frames,
                                uint32_t count, uint64_t sample_time);
void tascam_set_input_callback(tascam_engine *e, tascam_input_cb cb, void *ctx);

/* Callback optionnel (thread USB) juste avant l'envoi d'un bloc de sortie :
   le client peut modifier `frames` (sample_time = position du 1er). */
typedef void (*tascam_output_cb)(void *ctx, float *frames,
                                 uint32_t count, uint64_t sample_time);
void tascam_set_output_callback(tascam_engine *e, tascam_output_cb cb, void *ctx);

/* Callback optionnel : la carte a été débranchée. */
typedef void (*tascam_lost_cb)(void *ctx);
void tascam_set_lost_callback(tascam_engine *e, tascam_lost_cb cb, void *ctx);

/* Journalisation (par défaut : stderr). */
typedef void (*tascam_log_fn)(const char *msg);
void tascam_set_logger(tascam_log_fn fn);

#ifdef __cplusplus
}
#endif
#endif
