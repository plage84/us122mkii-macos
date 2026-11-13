/*
 * TascamPlugIn.c — plugin Core Audio (AudioServerPlugIn) pour la
 * TASCAM US-122 MKII sur Apple Silicon, sans driver Tascam.
 * Plage 84.
 *
 * Le plugin est chargé par coreaudiod. Il parle directement à la carte en
 * USB grâce au moteur tascam_usb.c (libusb liée statiquement) et expose un
 * périphérique "US-122 MKII" 2 entrées / 2 sorties, float32, à 44,1 / 48 /
 * 88,2 / 96 kHz. L'horloge est celle de la carte (dérivée du bus USB).
 *
 * Objets :
 *   1 = plugin, 2 = périphérique, 3 = flux d'entrée, 4 = flux de sortie
 */
#include <CoreAudio/AudioServerPlugIn.h>
#include <CoreFoundation/CoreFoundation.h>
#include <dispatch/dispatch.h>
#include <IOKit/IOMessage.h>
#include <IOKit/pwr_mgt/IOPMLib.h>
#include <os/log.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "tascam_usb.h"

/* ------------------------------------------------------------ identité */
#define kPlugIn_BundleID     "fr.plage84.tascam.us122mkii"
#define kDevice_Name         "US-122 MKII"
#define kDevice_Manufacturer "TASCAM (driver Plage 84)"
#define kDevice_UID          "Plage84_US122MKII"
#define kDevice_ModelUID     "Plage84_US122MKII_Model"

enum {
    kObjectID_PlugIn       = kAudioObjectPlugInObject,
    kObjectID_Device       = 2,
    kObjectID_Stream_Input = 3,
    kObjectID_Stream_Output= 4
};

#define kNumChannels 2
static const Float64 kRates[] = { 44100.0, 48000.0, 88200.0, 96000.0 };
#define kNumRates (sizeof kRates / sizeof kRates[0])

/* ------------------------------------------------------------ état */
static os_log_t                    gLog;
static AudioServerPlugInHostRef    gHost = NULL;
static UInt32                      gRefCount = 0;
static pthread_mutex_t             gStateMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t             gIOMutex = PTHREAD_MUTEX_INITIALIZER;

static atomic_int                  gPresent = 0;     /* carte branchée */
static Float64                     gRate = 48000.0;
static UInt32                      gIOCount = 0;
static tascam_engine              *gEngine = NULL;
static atomic_int                  gEngineOK = 0;    /* moteur prêt pour l'IO */
static Float64                     gLastZeroSample = 0;
static UInt64                      gLastZeroHost = 0;
static Boolean                     gInputActive = true, gOutputActive = true;
static atomic_int                  gWriteStats = 0;
static uint64_t                    gStopGen = 0;
static UInt64                      gSeed = 1;        /* change à chaque rupture de l'horloge */
static atomic_int                  gIOBusy = 0;      /* appels IO en cours sur le moteur */
static atomic_int                  gSleeping = 0;    /* Mac en veille */
static uint64_t                    gNextRetry = 0;   /* prochaine tentative de reprise (heure hôte) */
static uint64_t                    gStallLast = 0;   /* dernière position d'entrée vue */
static int                         gStallTicks = 0;
static void delayed_close_fn(void *ctx);

static void log_fn(const char *msg) { os_log(gLog, "%{public}s", msg); }

static UInt32 zero_period(Float64 rate) { return rate > 50000.0 ? 8192 : 4096; }
/* marge d'entrée : 16 ms. Avec 5 ms, macOS venait parfois lire du son pas
   encore arrivé (retards ponctuels du traitement USB) -> clics à l'enregistrement */
static UInt32 safety_in(Float64 rate)  { return (UInt32)(rate * 0.016); }
static UInt32 safety_out(Float64 rate) { return tascam_input_chunk_frames((int)rate) + 32; }
static UInt32 latency_out(Float64 rate){ return tascam_output_lead_frames((int)rate) + 2 * tascam_input_chunk_frames((int)rate); }

/* ------------------------------------------------------------ prototypes */
static HRESULT  PI_QueryInterface(void*, REFIID, LPVOID*);
static ULONG    PI_AddRef(void*);
static ULONG    PI_Release(void*);
static OSStatus PI_Initialize(AudioServerPlugInDriverRef, AudioServerPlugInHostRef);
static OSStatus PI_CreateDevice(AudioServerPlugInDriverRef, CFDictionaryRef, const AudioServerPlugInClientInfo*, AudioObjectID*);
static OSStatus PI_DestroyDevice(AudioServerPlugInDriverRef, AudioObjectID);
static OSStatus PI_AddDeviceClient(AudioServerPlugInDriverRef, AudioObjectID, const AudioServerPlugInClientInfo*);
static OSStatus PI_RemoveDeviceClient(AudioServerPlugInDriverRef, AudioObjectID, const AudioServerPlugInClientInfo*);
static OSStatus PI_PerformDeviceConfigurationChange(AudioServerPlugInDriverRef, AudioObjectID, UInt64, void*);
static OSStatus PI_AbortDeviceConfigurationChange(AudioServerPlugInDriverRef, AudioObjectID, UInt64, void*);
static Boolean  PI_HasProperty(AudioServerPlugInDriverRef, AudioObjectID, pid_t, const AudioObjectPropertyAddress*);
static OSStatus PI_IsPropertySettable(AudioServerPlugInDriverRef, AudioObjectID, pid_t, const AudioObjectPropertyAddress*, Boolean*);
static OSStatus PI_GetPropertyDataSize(AudioServerPlugInDriverRef, AudioObjectID, pid_t, const AudioObjectPropertyAddress*, UInt32, const void*, UInt32*);
static OSStatus PI_GetPropertyData(AudioServerPlugInDriverRef, AudioObjectID, pid_t, const AudioObjectPropertyAddress*, UInt32, const void*, UInt32, UInt32*, void*);
static OSStatus PI_SetPropertyData(AudioServerPlugInDriverRef, AudioObjectID, pid_t, const AudioObjectPropertyAddress*, UInt32, const void*, UInt32, const void*);
static OSStatus PI_StartIO(AudioServerPlugInDriverRef, AudioObjectID, UInt32);
static OSStatus PI_StopIO(AudioServerPlugInDriverRef, AudioObjectID, UInt32);
static OSStatus PI_GetZeroTimeStamp(AudioServerPlugInDriverRef, AudioObjectID, UInt32, Float64*, UInt64*, UInt64*);
static OSStatus PI_WillDoIOOperation(AudioServerPlugInDriverRef, AudioObjectID, UInt32, UInt32, Boolean*, Boolean*);
static OSStatus PI_BeginIOOperation(AudioServerPlugInDriverRef, AudioObjectID, UInt32, UInt32, UInt32, const AudioServerPlugInIOCycleInfo*);
static OSStatus PI_DoIOOperation(AudioServerPlugInDriverRef, AudioObjectID, AudioObjectID, UInt32, UInt32, UInt32, const AudioServerPlugInIOCycleInfo*, void*, void*);
static OSStatus PI_EndIOOperation(AudioServerPlugInDriverRef, AudioObjectID, UInt32, UInt32, UInt32, const AudioServerPlugInIOCycleInfo*);

static AudioServerPlugInDriverInterface gInterface = {
    NULL,
    PI_QueryInterface, PI_AddRef, PI_Release,
    PI_Initialize, PI_CreateDevice, PI_DestroyDevice,
    PI_AddDeviceClient, PI_RemoveDeviceClient,
    PI_PerformDeviceConfigurationChange, PI_AbortDeviceConfigurationChange,
    PI_HasProperty, PI_IsPropertySettable, PI_GetPropertyDataSize,
    PI_GetPropertyData, PI_SetPropertyData,
    PI_StartIO, PI_StopIO, PI_GetZeroTimeStamp,
    PI_WillDoIOOperation, PI_BeginIOOperation, PI_DoIOOperation, PI_EndIOOperation
};
static AudioServerPlugInDriverInterface *gInterfacePtr = &gInterface;
static AudioServerPlugInDriverRef gDriverRef = &gInterfacePtr;

/* ------------------------------------------------------------ fabrique */
void *TascamPlugIn_Create(CFAllocatorRef allocator, CFUUIDRef requestedTypeUUID);
void *TascamPlugIn_Create(CFAllocatorRef allocator, CFUUIDRef requestedTypeUUID)
{
    (void)allocator;
    if (CFEqual(requestedTypeUUID, kAudioServerPlugInTypeUUID)) return gDriverRef;
    return NULL;
}

static HRESULT PI_QueryInterface(void *inDriver, REFIID inUUID, LPVOID *outInterface)
{
    if (inDriver != gDriverRef || !outInterface) return kAudioHardwareBadObjectError;
    CFUUIDRef req = CFUUIDCreateFromUUIDBytes(NULL, inUUID);
    HRESULT r = E_NOINTERFACE;
    if (CFEqual(req, IUnknownUUID) || CFEqual(req, kAudioServerPlugInDriverInterfaceUUID)) {
        pthread_mutex_lock(&gStateMutex);
        gRefCount++;
        pthread_mutex_unlock(&gStateMutex);
        *outInterface = gDriverRef;
        r = S_OK;
    }
    CFRelease(req);
    return r;
}

static ULONG PI_AddRef(void *inDriver)
{
    if (inDriver != gDriverRef) return 0;
    pthread_mutex_lock(&gStateMutex);
    ULONG r = ++gRefCount;
    pthread_mutex_unlock(&gStateMutex);
    return r;
}

static ULONG PI_Release(void *inDriver)
{
    if (inDriver != gDriverRef) return 0;
    pthread_mutex_lock(&gStateMutex);
    if (gRefCount > 0) gRefCount--;
    ULONG r = gRefCount;
    pthread_mutex_unlock(&gStateMutex);
    return r;
}

/* ------------------------------------------------------------ notifications */
typedef struct { AudioObjectID obj; UInt32 n; AudioObjectPropertyAddress a[4]; } notif_t;

static void notify_fn(void *p)
{
    notif_t *n = (notif_t *)p;
    if (gHost) gHost->PropertiesChanged(gHost, n->obj, n->n, n->a);
    free(n);
}

static void notify_async(AudioObjectID obj, UInt32 count, const AudioObjectPropertySelector *sels)
{
    notif_t *n = (notif_t *)calloc(1, sizeof *n);
    if (!n) return;
    n->obj = obj;
    n->n = count > 4 ? 4 : count;
    for (UInt32 i = 0; i < n->n; i++) {
        n->a[i].mSelector = sels[i];
        n->a[i].mScope = kAudioObjectPropertyScopeGlobal;
        n->a[i].mElement = kAudioObjectPropertyElementMain;
    }
    dispatch_async_f(dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0), n, notify_fn);
}

typedef struct { UInt64 action; } cfg_t;
static void request_cfg_fn(void *p)
{
    cfg_t *c = (cfg_t *)p;
    if (gHost) gHost->RequestDeviceConfigurationChange(gHost, kObjectID_Device, c->action, NULL);
    free(c);
}

/* ------------------------------------------------------------ reprise automatique */
/* Attend qu'aucun appel IO n'utilise plus le moteur (gEngineOK doit déjà valoir 0). */
static void wait_io_idle(void)
{
    for (int i = 0; i < 2000 && atomic_load(&gIOBusy) > 0; i++) usleep(250);
}

/* Ferme le moteur courant. À appeler avec gIOMutex verrouillé. */
static void drop_engine_locked(const char *why)
{
    if (!gEngine) return;
    atomic_store(&gEngineOK, 0);
    wait_io_idle();
    os_log(gLog, "moteur fermé : %{public}s", why);
    tascam_close(gEngine);
    gEngine = NULL;
}

/* (Re)démarre le moteur pendant que des clients font de l'IO.
   L'horloge repart de zéro : on change la graine des horodatages pour que
   macOS se recale. À appeler avec gIOMutex verrouillé. */
static int restart_engine_locked(const char *why)
{
    char err[256] = "";
    drop_engine_locked(why);
    if (tascam_open(&gEngine, err, sizeof err) != 0) {
        gEngine = NULL;
        os_log_error(gLog, "reprise impossible (ouverture) : %{public}s", err);
        return -1;
    }
    if (tascam_start(gEngine, (int)gRate, err, sizeof err) != 0) {
        os_log_error(gLog, "reprise impossible (démarrage) : %{public}s", err);
        tascam_close(gEngine);
        gEngine = NULL;
        return -1;
    }
    gLastZeroSample = 0;
    gLastZeroHost = tascam_host_now();
    gSeed++;
    gStallLast = 0;
    gStallTicks = 0;
    atomic_store(&gEngineOK, 1);
    os_log(gLog, "carte relancée (%{public}s) à %.0f Hz", why, gRate);
    return 0;
}

/* Surveillance, appelée toutes les 0,5 s : relance la carte si le flux
   s'est arrêté (sortie de veille, carte rebranchée, erreur USB). */
static void watchdog_tick(void)
{
    uint64_t now = tascam_host_now();
    double tps = tascam_host_ticks_per_second();
    pthread_mutex_lock(&gIOMutex);
    if (atomic_load(&gSleeping)) { pthread_mutex_unlock(&gIOMutex); return; }

    if (gEngine) {
        tascam_stats st;
        tascam_get_stats(gEngine, &st);
        int dead = st.device_lost;
        if (tascam_is_running(gEngine)) {
            if (st.in_frames == gStallLast) gStallTicks++;
            else gStallTicks = 0;
            gStallLast = st.in_frames;
            if (gStallTicks >= 3) dead = 1;          /* plus rien depuis 1,5 s */
        }
        if (dead) {
            if (gIOCount > 0) {
                drop_engine_locked(st.device_lost ? "carte perdue" : "flux arrêté");
                gNextRetry = now + (uint64_t)(0.5 * tps);
            } else {
                drop_engine_locked("flux arrêté (sans client)");
            }
        }
    }
    /* des clients attendent du son mais le moteur est fermé : on relance */
    if (!gEngine && gIOCount > 0 && atomic_load(&gPresent) && now >= gNextRetry) {
        if (restart_engine_locked("reprise automatique") != 0)
            gNextRetry = now + (uint64_t)(2.0 * tps);
    }
    pthread_mutex_unlock(&gIOMutex);
}

/* ------------------------------------------------------------ veille du Mac */
static io_connect_t gPMRoot = 0;

static void power_cb(void *ref, io_service_t svc, natural_t type, void *arg)
{
    (void)ref; (void)svc;
    switch (type) {
    case kIOMessageCanSystemSleep:
        IOAllowPowerChange(gPMRoot, (long)arg);
        break;
    case kIOMessageSystemWillSleep:
        atomic_store(&gSleeping, 1);
        pthread_mutex_lock(&gIOMutex);
        drop_engine_locked("mise en veille");
        pthread_mutex_unlock(&gIOMutex);
        IOAllowPowerChange(gPMRoot, (long)arg);
        break;
    case kIOMessageSystemHasPoweredOn: {
        /* on laisse 2 s à l'USB pour se réveiller, la surveillance relance */
        pthread_mutex_lock(&gIOMutex);
        gNextRetry = tascam_host_now() + (uint64_t)(2.0 * tascam_host_ticks_per_second());
        pthread_mutex_unlock(&gIOMutex);
        atomic_store(&gSleeping, 0);
        os_log(gLog, "sortie de veille");
        break;
    }
    default: break;
    }
}

static void *power_thread(void *arg)
{
    (void)arg;
    IONotificationPortRef port = NULL;
    io_object_t notifier = 0;
    gPMRoot = IORegisterForSystemPower(NULL, &port, power_cb, &notifier);
    if (!gPMRoot) {
        os_log(gLog, "notifications de veille indisponibles : la surveillance du flux prend le relais");
        return NULL;
    }
    CFRunLoopAddSource(CFRunLoopGetCurrent(), IONotificationPortGetRunLoopSource(port),
                       kCFRunLoopDefaultMode);
    os_log(gLog, "notifications de veille actives");
    CFRunLoopRun();
    return NULL;
}

/* ------------------------------------------------------------ détection carte */
static void *presence_thread(void *arg)
{
    (void)arg;
    for (unsigned tick = 0;; tick++) {
        if (tick > 0) {
            usleep(500000);
            watchdog_tick();                 /* toutes les 0,5 s */
        }
        if (tick % 2) continue;              /* présence : toutes les secondes */
        int now = atomic_load(&gSleeping) ? atomic_load(&gPresent) : tascam_device_present();
        if (now != atomic_load(&gPresent)) {
            atomic_store(&gPresent, now);
            os_log(gLog, "US-122 MKII %{public}s", now ? "branchée" : "débranchée");
            AudioObjectPropertySelector s[2] = { kAudioPlugInPropertyDeviceList,
                                                 kAudioObjectPropertyOwnedObjects };
            notify_async(kObjectID_PlugIn, 2, s);
        }
        if (tick % 4) continue;              /* suivi : toutes les 2 s */
        pthread_mutex_lock(&gIOMutex);
        if (gEngine && atomic_load(&gEngineOK)) {
            tascam_stats st;
            tascam_get_stats(gEngine, &st);
            Float64 zs = gLastZeroSample;
            os_log_debug(gLog, "suivi : in %llu out %llu, %.1f Hz, dérive %lld, ajustements %u, vides %llu, file %u, report %lld, recalages %llu, trous %llu, "
                   "erreurs %llu/%llu, lectures tardives %llu (pire manque %llu trames), entrée anormale %llu, horodatage %.0f",
                   (unsigned long long)st.in_frames, (unsigned long long)st.out_frames,
                   st.measured_rate, (long long)st.lead_drift, st.fifo_depth,
                   (unsigned long long)st.zero_packets, st.depth, (long long)st.carry,
                   (unsigned long long)st.realigns, (unsigned long long)st.resyncs,
                   (unsigned long long)st.in_errors, (unsigned long long)st.out_errors,
                   (unsigned long long)st.late_reads, (unsigned long long)st.late_max,
                   (unsigned long long)st.anomalies, zs);
            gWriteStats = 1;
        }
        pthread_mutex_unlock(&gIOMutex);
    }
    return NULL;
}

static OSStatus PI_Initialize(AudioServerPlugInDriverRef inDriver, AudioServerPlugInHostRef inHost)
{
    if (inDriver != gDriverRef) return kAudioHardwareBadObjectError;
    gHost = inHost;
    gLog = os_log_create(kPlugIn_BundleID, "driver");
    tascam_set_logger(log_fn);
    tascam_host_ticks_per_second();
    atomic_store(&gPresent, tascam_device_present());
    os_log(gLog, "plugin chargé, carte %{public}s",
           atomic_load(&gPresent) ? "présente" : "absente");
    pthread_t t;
    if (pthread_create(&t, NULL, presence_thread, NULL) == 0) pthread_detach(t);
    if (pthread_create(&t, NULL, power_thread, NULL) == 0) pthread_detach(t);
    return kAudioHardwareNoError;
}

static OSStatus PI_CreateDevice(AudioServerPlugInDriverRef d, CFDictionaryRef desc, const AudioServerPlugInClientInfo *ci, AudioObjectID *out)
{ (void)d; (void)desc; (void)ci; (void)out; return kAudioHardwareUnsupportedOperationError; }
static OSStatus PI_DestroyDevice(AudioServerPlugInDriverRef d, AudioObjectID id)
{ (void)d; (void)id; return kAudioHardwareUnsupportedOperationError; }
static OSStatus PI_AddDeviceClient(AudioServerPlugInDriverRef d, AudioObjectID id, const AudioServerPlugInClientInfo *ci)
{ (void)ci; return (d == gDriverRef && id == kObjectID_Device) ? kAudioHardwareNoError : kAudioHardwareBadObjectError; }
static OSStatus PI_RemoveDeviceClient(AudioServerPlugInDriverRef d, AudioObjectID id, const AudioServerPlugInClientInfo *ci)
{ (void)ci; return (d == gDriverRef && id == kObjectID_Device) ? kAudioHardwareNoError : kAudioHardwareBadObjectError; }

static int valid_rate(Float64 r)
{
    for (size_t i = 0; i < kNumRates; i++) if (kRates[i] == r) return 1;
    return 0;
}

static OSStatus PI_PerformDeviceConfigurationChange(AudioServerPlugInDriverRef d, AudioObjectID id, UInt64 action, void *info)
{
    (void)info;
    if (d != gDriverRef || id != kObjectID_Device) return kAudioHardwareBadObjectError;
    if (!valid_rate((Float64)action)) return kAudioHardwareIllegalOperationError;
    pthread_mutex_lock(&gIOMutex);
    gRate = (Float64)action;
    if (gEngine && tascam_is_running(gEngine)) {
        char err[256];
        atomic_store(&gEngineOK, 0);
        wait_io_idle();
        if (tascam_start(gEngine, (int)gRate, err, sizeof err) == 0) atomic_store(&gEngineOK, 1);
        else os_log_error(gLog, "changement de fréquence : %{public}s", err);
        gLastZeroSample = 0;
        gLastZeroHost = 0;
        gSeed++;
    } else if (gIOCount > 0 && atomic_load(&gPresent)) {
        restart_engine_locked("changement de fréquence");
    }
    pthread_mutex_unlock(&gIOMutex);
    os_log(gLog, "fréquence : %.0f Hz", gRate);
    return kAudioHardwareNoError;
}

static OSStatus PI_AbortDeviceConfigurationChange(AudioServerPlugInDriverRef d, AudioObjectID id, UInt64 a, void *i)
{ (void)a; (void)i; return (d == gDriverRef && id == kObjectID_Device) ? kAudioHardwareNoError : kAudioHardwareBadObjectError; }

/* ------------------------------------------------------------ propriétés */
static int object_exists(AudioObjectID id)
{
    if (id == kObjectID_PlugIn) return 1;
    if (id == kObjectID_Device || id == kObjectID_Stream_Input || id == kObjectID_Stream_Output)
        return atomic_load(&gPresent);
    return 0;
}

static void fill_asbd(AudioStreamBasicDescription *f, Float64 rate)
{
    memset(f, 0, sizeof *f);
    f->mSampleRate = rate;
    f->mFormatID = kAudioFormatLinearPCM;
    f->mFormatFlags = kAudioFormatFlagIsFloat | kAudioFormatFlagsNativeEndian | kAudioFormatFlagIsPacked;
    f->mBytesPerPacket = 4 * kNumChannels;
    f->mFramesPerPacket = 1;
    f->mBytesPerFrame = 4 * kNumChannels;
    f->mChannelsPerFrame = kNumChannels;
    f->mBitsPerChannel = 32;
}

static UInt32 layout_size(void)
{
    return (UInt32)(offsetof(AudioChannelLayout, mChannelDescriptions) +
                    kNumChannels * sizeof(AudioChannelDescription));
}

static Boolean PI_HasProperty(AudioServerPlugInDriverRef d, AudioObjectID id, pid_t pid, const AudioObjectPropertyAddress *a)
{
    (void)pid;
    if (d != gDriverRef || !a || !object_exists(id)) return false;
    UInt32 size;
    /* une propriété existe si on sait donner sa taille */
    return PI_GetPropertyDataSize(d, id, 0, a, 0, NULL, &size) == kAudioHardwareNoError;
}

static OSStatus PI_IsPropertySettable(AudioServerPlugInDriverRef d, AudioObjectID id, pid_t pid, const AudioObjectPropertyAddress *a, Boolean *out)
{
    (void)pid;
    if (d != gDriverRef || !a || !out) return kAudioHardwareIllegalOperationError;
    if (!object_exists(id)) return kAudioHardwareBadObjectError;
    UInt32 size;
    OSStatus r = PI_GetPropertyDataSize(d, id, 0, a, 0, NULL, &size);
    if (r) return r;
    *out = false;
    if (id == kObjectID_Device && a->mSelector == kAudioDevicePropertyNominalSampleRate) *out = true;
    if ((id == kObjectID_Stream_Input || id == kObjectID_Stream_Output) &&
        (a->mSelector == kAudioStreamPropertyIsActive ||
         a->mSelector == kAudioStreamPropertyVirtualFormat ||
         a->mSelector == kAudioStreamPropertyPhysicalFormat)) *out = true;
    return kAudioHardwareNoError;
}

static UInt32 device_stream_count(AudioObjectPropertyScope scope)
{
    if (scope == kAudioObjectPropertyScopeInput || scope == kAudioObjectPropertyScopeOutput) return 1;
    return 2;
}

static OSStatus PI_GetPropertyDataSize(AudioServerPlugInDriverRef d, AudioObjectID id, pid_t pid, const AudioObjectPropertyAddress *a, UInt32 qs, const void *q, UInt32 *out)
{
    (void)pid; (void)qs; (void)q;
    if (d != gDriverRef || !a || !out) return kAudioHardwareIllegalOperationError;
    if (!object_exists(id)) return kAudioHardwareBadObjectError;
    int present = atomic_load(&gPresent);

    if (id == kObjectID_PlugIn) {
        switch (a->mSelector) {
        case kAudioObjectPropertyBaseClass:
        case kAudioObjectPropertyClass:      *out = sizeof(AudioClassID); return 0;
        case kAudioObjectPropertyOwner:      *out = sizeof(AudioObjectID); return 0;
        case kAudioObjectPropertyManufacturer: *out = sizeof(CFStringRef); return 0;
        case kAudioObjectPropertyOwnedObjects:
        case kAudioPlugInPropertyDeviceList: *out = present ? sizeof(AudioObjectID) : 0; return 0;
        case kAudioPlugInPropertyTranslateUIDToDevice: *out = sizeof(AudioObjectID); return 0;
        case kAudioPlugInPropertyResourceBundle: *out = sizeof(CFStringRef); return 0;
        }
        return kAudioHardwareUnknownPropertyError;
    }

    if (id == kObjectID_Device) {
        switch (a->mSelector) {
        case kAudioObjectPropertyBaseClass:
        case kAudioObjectPropertyClass:      *out = sizeof(AudioClassID); return 0;
        case kAudioObjectPropertyOwner:      *out = sizeof(AudioObjectID); return 0;
        case kAudioObjectPropertyName:
        case kAudioObjectPropertyManufacturer:
        case kAudioDevicePropertyDeviceUID:
        case kAudioDevicePropertyModelUID:   *out = sizeof(CFStringRef); return 0;
        case kAudioDevicePropertyTransportType:
        case kAudioDevicePropertyClockDomain:
        case kAudioDevicePropertyDeviceIsAlive:
        case kAudioDevicePropertyDeviceIsRunning:
        case kAudioDevicePropertyDeviceCanBeDefaultDevice:
        case kAudioDevicePropertyDeviceCanBeDefaultSystemDevice:
        case kAudioDevicePropertyLatency:
        case kAudioDevicePropertySafetyOffset:
        case kAudioDevicePropertyIsHidden:
        case kAudioDevicePropertyZeroTimeStampPeriod: *out = sizeof(UInt32); return 0;
        case kAudioDevicePropertyRelatedDevices: *out = sizeof(AudioObjectID); return 0;
        case kAudioObjectPropertyOwnedObjects:
        case kAudioDevicePropertyStreams:
            *out = device_stream_count(a->mScope) * sizeof(AudioObjectID); return 0;
        case kAudioObjectPropertyControlList: *out = 0; return 0;
        case kAudioDevicePropertyNominalSampleRate: *out = sizeof(Float64); return 0;
        case kAudioDevicePropertyAvailableNominalSampleRates: *out = kNumRates * sizeof(AudioValueRange); return 0;
        case kAudioDevicePropertyPreferredChannelsForStereo: *out = 2 * sizeof(UInt32); return 0;
        case kAudioDevicePropertyPreferredChannelLayout: *out = layout_size(); return 0;
        }
        return kAudioHardwareUnknownPropertyError;
    }

    /* flux */
    switch (a->mSelector) {
    case kAudioObjectPropertyBaseClass:
    case kAudioObjectPropertyClass:      *out = sizeof(AudioClassID); return 0;
    case kAudioObjectPropertyOwner:      *out = sizeof(AudioObjectID); return 0;
    case kAudioObjectPropertyOwnedObjects: *out = 0; return 0;
    case kAudioStreamPropertyIsActive:
    case kAudioStreamPropertyDirection:
    case kAudioStreamPropertyTerminalType:
    case kAudioStreamPropertyStartingChannel:
    case kAudioStreamPropertyLatency:    *out = sizeof(UInt32); return 0;
    case kAudioStreamPropertyVirtualFormat:
    case kAudioStreamPropertyPhysicalFormat: *out = sizeof(AudioStreamBasicDescription); return 0;
    case kAudioStreamPropertyAvailableVirtualFormats:
    case kAudioStreamPropertyAvailablePhysicalFormats:
        *out = kNumRates * sizeof(AudioStreamRangedDescription); return 0;
    }
    return kAudioHardwareUnknownPropertyError;
}

#define NEED(sz) do { if (inSize < (sz)) return kAudioHardwareBadPropertySizeError; } while (0)
#define PUT(type, val) do { NEED(sizeof(type)); *(type *)out = (val); *outSize = sizeof(type); return 0; } while (0)

static OSStatus PI_GetPropertyData(AudioServerPlugInDriverRef d, AudioObjectID id, pid_t pid, const AudioObjectPropertyAddress *a, UInt32 qs, const void *q, UInt32 inSize, UInt32 *outSize, void *out)
{
    (void)pid;
    if (d != gDriverRef || !a || !outSize || !out) return kAudioHardwareIllegalOperationError;
    if (!object_exists(id)) return kAudioHardwareBadObjectError;
    int present = atomic_load(&gPresent);
    Float64 rate = gRate;

    if (id == kObjectID_PlugIn) {
        switch (a->mSelector) {
        case kAudioObjectPropertyBaseClass: PUT(AudioClassID, kAudioObjectClassID);
        case kAudioObjectPropertyClass:     PUT(AudioClassID, kAudioPlugInClassID);
        case kAudioObjectPropertyOwner:     PUT(AudioObjectID, kAudioObjectUnknown);
        case kAudioObjectPropertyManufacturer: PUT(CFStringRef, CFSTR("Plage 84"));
        case kAudioObjectPropertyOwnedObjects:
        case kAudioPlugInPropertyDeviceList:
            if (present && inSize >= sizeof(AudioObjectID)) {
                *(AudioObjectID *)out = kObjectID_Device;
                *outSize = sizeof(AudioObjectID);
            } else *outSize = 0;
            return 0;
        case kAudioPlugInPropertyTranslateUIDToDevice: {
            NEED(sizeof(AudioObjectID));
            AudioObjectID r = kAudioObjectUnknown;
            if (present && qs == sizeof(CFStringRef) && q) {
                CFStringRef uid = *(const CFStringRef *)q;
                if (uid && CFStringCompare(uid, CFSTR(kDevice_UID), 0) == kCFCompareEqualTo)
                    r = kObjectID_Device;
            }
            *(AudioObjectID *)out = r;
            *outSize = sizeof(AudioObjectID);
            return 0;
        }
        case kAudioPlugInPropertyResourceBundle: PUT(CFStringRef, CFSTR(""));
        }
        return kAudioHardwareUnknownPropertyError;
    }

    if (id == kObjectID_Device) {
        AudioObjectPropertyScope sc = a->mScope;
        switch (a->mSelector) {
        case kAudioObjectPropertyBaseClass: PUT(AudioClassID, kAudioObjectClassID);
        case kAudioObjectPropertyClass:     PUT(AudioClassID, kAudioDeviceClassID);
        case kAudioObjectPropertyOwner:     PUT(AudioObjectID, kObjectID_PlugIn);
        case kAudioObjectPropertyName:      PUT(CFStringRef, CFSTR(kDevice_Name));
        case kAudioObjectPropertyManufacturer: PUT(CFStringRef, CFSTR(kDevice_Manufacturer));
        case kAudioDevicePropertyDeviceUID: PUT(CFStringRef, CFSTR(kDevice_UID));
        case kAudioDevicePropertyModelUID:  PUT(CFStringRef, CFSTR(kDevice_ModelUID));
        case kAudioDevicePropertyTransportType: PUT(UInt32, kAudioDeviceTransportTypeUSB);
        case kAudioDevicePropertyRelatedDevices: PUT(AudioObjectID, kObjectID_Device);
        case kAudioDevicePropertyClockDomain: PUT(UInt32, 0);
        case kAudioDevicePropertyDeviceIsAlive: PUT(UInt32, 1);
        case kAudioDevicePropertyDeviceIsRunning: PUT(UInt32, gIOCount > 0 ? 1 : 0);
        case kAudioDevicePropertyDeviceCanBeDefaultDevice: PUT(UInt32, 1);
        case kAudioDevicePropertyDeviceCanBeDefaultSystemDevice: PUT(UInt32, 1);
        case kAudioDevicePropertyLatency:
            PUT(UInt32, sc == kAudioObjectPropertyScopeOutput ? latency_out(rate) : 0);
        case kAudioDevicePropertySafetyOffset:
            PUT(UInt32, sc == kAudioObjectPropertyScopeInput ? safety_in(rate) : safety_out(rate));
        case kAudioDevicePropertyIsHidden: PUT(UInt32, 0);
        case kAudioDevicePropertyZeroTimeStampPeriod: PUT(UInt32, zero_period(rate));
        case kAudioObjectPropertyOwnedObjects:
        case kAudioDevicePropertyStreams: {
            AudioObjectID ids[2];
            UInt32 n = 0;
            if (sc != kAudioObjectPropertyScopeOutput) ids[n++] = kObjectID_Stream_Input;
            if (sc != kAudioObjectPropertyScopeInput) ids[n++] = kObjectID_Stream_Output;
            UInt32 max = inSize / sizeof(AudioObjectID);
            if (n > max) n = max;
            memcpy(out, ids, n * sizeof(AudioObjectID));
            *outSize = n * sizeof(AudioObjectID);
            return 0;
        }
        case kAudioObjectPropertyControlList: *outSize = 0; return 0;
        case kAudioDevicePropertyNominalSampleRate: PUT(Float64, rate);
        case kAudioDevicePropertyAvailableNominalSampleRates: {
            UInt32 n = inSize / sizeof(AudioValueRange);
            if (n > kNumRates) n = kNumRates;
            AudioValueRange *r = (AudioValueRange *)out;
            for (UInt32 i = 0; i < n; i++) { r[i].mMinimum = kRates[i]; r[i].mMaximum = kRates[i]; }
            *outSize = n * sizeof(AudioValueRange);
            return 0;
        }
        case kAudioDevicePropertyPreferredChannelsForStereo: {
            NEED(2 * sizeof(UInt32));
            ((UInt32 *)out)[0] = 1;
            ((UInt32 *)out)[1] = 2;
            *outSize = 2 * sizeof(UInt32);
            return 0;
        }
        case kAudioDevicePropertyPreferredChannelLayout: {
            UInt32 sz = layout_size();
            NEED(sz);
            AudioChannelLayout *l = (AudioChannelLayout *)out;
            memset(l, 0, sz);
            l->mChannelLayoutTag = kAudioChannelLayoutTag_UseChannelDescriptions;
            l->mNumberChannelDescriptions = kNumChannels;
            l->mChannelDescriptions[0].mChannelLabel = kAudioChannelLabel_Left;
            l->mChannelDescriptions[1].mChannelLabel = kAudioChannelLabel_Right;
            *outSize = sz;
            return 0;
        }
        }
        return kAudioHardwareUnknownPropertyError;
    }

    /* flux */
    Boolean isIn = (id == kObjectID_Stream_Input);
    switch (a->mSelector) {
    case kAudioObjectPropertyBaseClass: PUT(AudioClassID, kAudioObjectClassID);
    case kAudioObjectPropertyClass:     PUT(AudioClassID, kAudioStreamClassID);
    case kAudioObjectPropertyOwner:     PUT(AudioObjectID, kObjectID_Device);
    case kAudioObjectPropertyOwnedObjects: *outSize = 0; return 0;
    case kAudioStreamPropertyIsActive:  PUT(UInt32, isIn ? gInputActive : gOutputActive);
    case kAudioStreamPropertyDirection: PUT(UInt32, isIn ? 1 : 0);
    case kAudioStreamPropertyTerminalType: PUT(UInt32, kAudioStreamTerminalTypeLine);
    case kAudioStreamPropertyStartingChannel: PUT(UInt32, 1);
    case kAudioStreamPropertyLatency:   PUT(UInt32, 0);
    case kAudioStreamPropertyVirtualFormat:
    case kAudioStreamPropertyPhysicalFormat: {
        NEED(sizeof(AudioStreamBasicDescription));
        fill_asbd((AudioStreamBasicDescription *)out, rate);
        *outSize = sizeof(AudioStreamBasicDescription);
        return 0;
    }
    case kAudioStreamPropertyAvailableVirtualFormats:
    case kAudioStreamPropertyAvailablePhysicalFormats: {
        UInt32 n = inSize / sizeof(AudioStreamRangedDescription);
        if (n > kNumRates) n = kNumRates;
        AudioStreamRangedDescription *r = (AudioStreamRangedDescription *)out;
        for (UInt32 i = 0; i < n; i++) {
            fill_asbd(&r[i].mFormat, kRates[i]);
            r[i].mSampleRateRange.mMinimum = kRates[i];
            r[i].mSampleRateRange.mMaximum = kRates[i];
        }
        *outSize = n * sizeof(AudioStreamRangedDescription);
        return 0;
    }
    }
    return kAudioHardwareUnknownPropertyError;
}

static OSStatus request_rate(Float64 r)
{
    if (!valid_rate(r)) return kAudioHardwareIllegalOperationError;
    if (r == gRate) return 0;
    cfg_t *c = (cfg_t *)calloc(1, sizeof *c);
    if (!c) return kAudioHardwareUnspecifiedError;
    c->action = (UInt64)r;
    dispatch_async_f(dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0), c, request_cfg_fn);
    return 0;
}

static OSStatus PI_SetPropertyData(AudioServerPlugInDriverRef d, AudioObjectID id, pid_t pid, const AudioObjectPropertyAddress *a, UInt32 qs, const void *q, UInt32 inSize, const void *in)
{
    (void)pid; (void)qs; (void)q;
    if (d != gDriverRef || !a || !in) return kAudioHardwareIllegalOperationError;
    if (!object_exists(id)) return kAudioHardwareBadObjectError;

    if (id == kObjectID_Device && a->mSelector == kAudioDevicePropertyNominalSampleRate) {
        if (inSize < sizeof(Float64)) return kAudioHardwareBadPropertySizeError;
        return request_rate(*(const Float64 *)in);
    }
    if (id == kObjectID_Stream_Input || id == kObjectID_Stream_Output) {
        Boolean isIn = (id == kObjectID_Stream_Input);
        switch (a->mSelector) {
        case kAudioStreamPropertyIsActive: {
            if (inSize < sizeof(UInt32)) return kAudioHardwareBadPropertySizeError;
            Boolean v = *(const UInt32 *)in != 0;
            if (isIn) gInputActive = v; else gOutputActive = v;
            AudioObjectPropertySelector s = kAudioStreamPropertyIsActive;
            notify_async(id, 1, &s);
            return 0;
        }
        case kAudioStreamPropertyVirtualFormat:
        case kAudioStreamPropertyPhysicalFormat: {
            if (inSize < sizeof(AudioStreamBasicDescription)) return kAudioHardwareBadPropertySizeError;
            const AudioStreamBasicDescription *f = (const AudioStreamBasicDescription *)in;
            if (f->mFormatID != kAudioFormatLinearPCM || f->mChannelsPerFrame != kNumChannels ||
                f->mBitsPerChannel != 32 || !(f->mFormatFlags & kAudioFormatFlagIsFloat))
                return kAudioDeviceUnsupportedFormatError;
            return request_rate(f->mSampleRate);
        }
        }
    }
    return kAudioHardwareUnknownPropertyError;
}

/* ------------------------------------------------------------ IO */
static OSStatus PI_StartIO(AudioServerPlugInDriverRef d, AudioObjectID id, UInt32 client)
{
    (void)client;
    if (d != gDriverRef || id != kObjectID_Device) return kAudioHardwareBadObjectError;
    if (!atomic_load(&gPresent)) return kAudioHardwareNotRunningError;
    OSStatus res = 0;
    pthread_mutex_lock(&gIOMutex);
    if (gIOCount == 0) {
        char err[256] = "";
        gStopGen++;                          /* annule une fermeture différée */
        int reuse = 0;
        if (gEngine) {
            tascam_stats st0;
            tascam_get_stats(gEngine, &st0);
            if (st0.device_lost) drop_engine_locked("carte perdue");
            else reuse = tascam_is_running(gEngine) && tascam_rate(gEngine) == (int)gRate;
        }
        if (reuse) {
            /* la carte tourne encore : on reprend sans redémarrage (pas de clic) */
            atomic_store(&gEngineOK, 1);
            os_log(gLog, "IO reprise sans redémarrage");
        } else if (!gEngine && tascam_open(&gEngine, err, sizeof err) != 0) {
            gEngine = NULL;
            os_log_error(gLog, "ouverture : %{public}s", err);
            res = kAudioHardwareNotRunningError;
        } else if (tascam_start(gEngine, (int)gRate, err, sizeof err) != 0) {
            os_log_error(gLog, "démarrage : %{public}s", err);
            tascam_close(gEngine);
            gEngine = NULL;
            res = kAudioHardwareNotRunningError;
        } else {
            gLastZeroSample = 0;
            gLastZeroHost = tascam_host_now();
            gSeed++;
            gStallLast = 0;
            gStallTicks = 0;
            atomic_store(&gEngineOK, 1);
            os_log(gLog, "IO démarrée à %.0f Hz", gRate);
        }
    }
    if (res == 0) gIOCount++;
    pthread_mutex_unlock(&gIOMutex);
    if (res == 0) {
        AudioObjectPropertySelector s = kAudioDevicePropertyDeviceIsRunning;
        notify_async(kObjectID_Device, 1, &s);
    }
    return res;
}

static OSStatus PI_StopIO(AudioServerPlugInDriverRef d, AudioObjectID id, UInt32 client)
{
    (void)client;
    if (d != gDriverRef || id != kObjectID_Device) return kAudioHardwareBadObjectError;
    pthread_mutex_lock(&gIOMutex);
    if (gIOCount > 0) gIOCount--;
    if (gIOCount == 0 && gEngine) {
        /* on garde la carte allumée 10 s : entre deux morceaux, macOS
           arrête puis relance l'IO, et un redémarrage complet ferait un clic */
        uint64_t gen = ++gStopGen;
        dispatch_after_f(dispatch_time(DISPATCH_TIME_NOW, 10 * NSEC_PER_SEC),
                         dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0),
                         (void *)(uintptr_t)gen, delayed_close_fn);
    }
    pthread_mutex_unlock(&gIOMutex);
    AudioObjectPropertySelector s = kAudioDevicePropertyDeviceIsRunning;
    notify_async(kObjectID_Device, 1, &s);
    return 0;
}

static void delayed_close_fn(void *ctx)
{
    uint64_t gen = (uint64_t)(uintptr_t)ctx;
    pthread_mutex_lock(&gIOMutex);
    if (gen == gStopGen && gIOCount == 0 && gEngine) {
        atomic_store(&gEngineOK, 0);
        wait_io_idle();
        tascam_stats st;
        tascam_get_stats(gEngine, &st);
        os_log(gLog, "IO arrêtée : %llu trames, %.2f Hz mesurés, erreurs in/out %llu/%llu, "
               "sauts %llu, lectures tardives %llu",
               (unsigned long long)st.in_frames, st.measured_rate,
               (unsigned long long)st.in_errors, (unsigned long long)st.out_errors,
               (unsigned long long)st.out_skipped, (unsigned long long)st.late_reads);
        tascam_close(gEngine);   /* libère la carte pour d'autres programmes */
        gEngine = NULL;
        os_log(gLog, "carte libérée");
    }
    pthread_mutex_unlock(&gIOMutex);
}

static OSStatus PI_GetZeroTimeStamp(AudioServerPlugInDriverRef d, AudioObjectID id, UInt32 client, Float64 *outSample, UInt64 *outHost, UInt64 *outSeed)
{
    (void)client;
    if (d != gDriverRef || id != kObjectID_Device) return kAudioHardwareBadObjectError;
    double s;
    uint64_t h;
    atomic_fetch_add(&gIOBusy, 1);
    tascam_engine *e = gEngine;
    if (atomic_load(&gEngineOK) && e && tascam_zero_timestamp(e, zero_period(gRate), &s, &h)) {
        if (s >= gLastZeroSample) {     /* toujours croissant */
            gLastZeroSample = s;
            gLastZeroHost = h;
        }
    } else if (!atomic_load(&gEngineOK)) {
        /* carte en cours de relance : l'horloge avance au rythme nominal
           pour que macOS ne se bloque pas */
        uint64_t now = tascam_host_now();
        double period_ticks = zero_period(gRate) / gRate * tascam_host_ticks_per_second();
        while (gLastZeroHost && (double)(now - gLastZeroHost) >= period_ticks) {
            gLastZeroSample += zero_period(gRate);
            gLastZeroHost += (uint64_t)period_ticks;
        }
    }
    atomic_fetch_sub(&gIOBusy, 1);
    *outSample = gLastZeroSample;
    *outHost = gLastZeroHost;
    *outSeed = gSeed;
    return 0;
}

static OSStatus PI_WillDoIOOperation(AudioServerPlugInDriverRef d, AudioObjectID id, UInt32 client, UInt32 op, Boolean *willDo, Boolean *inPlace)
{
    (void)client;
    if (d != gDriverRef || id != kObjectID_Device) return kAudioHardwareBadObjectError;
    Boolean w = (op == kAudioServerPlugInIOOperationReadInput || op == kAudioServerPlugInIOOperationWriteMix);
    if (willDo) *willDo = w;
    if (inPlace) *inPlace = true;
    return 0;
}

static OSStatus PI_BeginIOOperation(AudioServerPlugInDriverRef d, AudioObjectID id, UInt32 c, UInt32 op, UInt32 n, const AudioServerPlugInIOCycleInfo *ci)
{ (void)c; (void)op; (void)n; (void)ci; return (d == gDriverRef && id == kObjectID_Device) ? 0 : kAudioHardwareBadObjectError; }

static OSStatus PI_EndIOOperation(AudioServerPlugInDriverRef d, AudioObjectID id, UInt32 c, UInt32 op, UInt32 n, const AudioServerPlugInIOCycleInfo *ci)
{ (void)c; (void)op; (void)n; (void)ci; return (d == gDriverRef && id == kObjectID_Device) ? 0 : kAudioHardwareBadObjectError; }

static OSStatus PI_DoIOOperation(AudioServerPlugInDriverRef d, AudioObjectID id, AudioObjectID stream, UInt32 client, UInt32 op, UInt32 frames, const AudioServerPlugInIOCycleInfo *ci, void *mainBuf, void *secBuf)
{
    (void)client; (void)secBuf;
    if (d != gDriverRef || id != kObjectID_Device) return kAudioHardwareBadObjectError;
    atomic_fetch_add(&gIOBusy, 1);
    tascam_engine *e = gEngine;
    int ok = atomic_load(&gEngineOK) && e;

    if (op == kAudioServerPlugInIOOperationReadInput && stream == kObjectID_Stream_Input) {
        Float64 st = ci->mInputTime.mSampleTime;
        if (ok && st >= 0)
            tascam_read_input(e, (uint64_t)st, (float *)mainBuf, frames);
        else
            memset(mainBuf, 0, frames * kNumChannels * sizeof(float));
    } else if (op == kAudioServerPlugInIOOperationWriteMix && stream == kObjectID_Stream_Output) {
        Float64 st = ci->mOutputTime.mSampleTime;
        if (ok && atomic_exchange(&gWriteStats, 0)) {
            /* position d'écriture de macOS comparée à celle du moteur */
            tascam_stats s2;
            tascam_get_stats(e, &s2);
            os_log_debug(gLog, "IO : macOS écrit à %.0f (taille %u), lit à %.0f ; moteur entrée %llu",
                   st, frames, ci->mInputTime.mSampleTime, (unsigned long long)s2.in_frames);
        }
        if (ok && st >= 0)
            tascam_write_output(e, (uint64_t)st, (const float *)mainBuf, frames);
    }
    atomic_fetch_sub(&gIOBusy, 1);
    return 0;
}
