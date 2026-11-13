# us122mkii-macos

An open-source Core Audio driver for the **TASCAM US-122 MKII** on **Apple Silicon Macs** (M1 – M4).

Tascam never released an Apple Silicon driver for this interface. This one runs entirely in user
space as an AudioServerPlugIn, so it needs **no kernel extension, no Tascam software and no
Apple developer account**. Once installed it is plug and play: plug the interface in and it shows
up in System Settings › Sound and in your DAW, like any class-compliant interface.

*Version française plus bas.*

> Unofficial project, not affiliated with or endorsed by TASCAM / TEAC.

## Status (v0.2.0)

| | |
|---|---|
| Playback, 2 channels | ✅ |
| Recording, 2 channels | ✅ |
| 44.1 / 48 / 88.2 / 96 kHz | ✅ |
| Sleep / wake | ✅ the interface restarts by itself |
| Unplug / replug | ✅ |
| MIDI in/out | ❌ not yet |
| Tested on | MacBook Pro M4 Pro, macOS 15.6, Ableton Live 12 |

Output latency is about 32 ms: macOS schedules isochronous USB transfers late when they come from
user space, so a deeper output queue is needed to avoid clicks.

## Install

You need the Xcode Command Line Tools and Homebrew's `libusb` (only to build; the installed driver
does not depend on Homebrew).

```bash
xcode-select --install          # if not installed yet
brew install libusb
```

Download this repository (green **Code** button › **Download ZIP**, or `git clone`), then in Terminal:

```bash
cd ~/Downloads/us122mkii-macos-main
bash build.sh       # compiles build/US122MKII.driver
bash install.sh     # asks for your admin password, restarts Core Audio
```

Plug in the US-122 MKII: it appears as **US-122 MKII**.
To remove the driver: `bash uninstall.sh`.

The driver is ad-hoc signed (not notarised); building it on your own Mac is what lets Core Audio load it.

## Tips

- The front-panel input knobs are a single continuous gain control. "LINE" and "MIC" on the scale
  only show the recommended zone. Aim for peaks around −10 to −6 dBFS in your DAW instead of
  boosting the track afterwards, or you will raise the noise floor with it.

## Troubleshooting

| Script | Purpose |
|---|---|
| `scripts/journal.sh` | Save the driver log to `journal.txt` |
| `scripts/diag.sh` | Diagnose a driver that does not show up |
| `scripts/tester.sh` | Test the USB engine alone (beep, recording, clock), driver not needed |

Live log: `log stream --level debug --predicate 'subsystem == "fr.plage84.tascam.us122mkii"'`

## How it works

- `src/tascam_usb.c` — USB engine built on libusb: duplex isochronous streaming, card-driven clock,
  24-bit ↔ float conversion, implicit feedback (output packet sizes mirror input sizes).
- `src/TascamPlugIn.c` — the AudioServerPlugIn: one device with a 2-channel input stream and a
  2-channel output stream, zero timestamps derived from the USB frame counter, automatic restart
  after sleep, unplugging or a stalled stream.
- `src/tascam_test.c` — command-line test tool.
- `Info.plist` — declares `AudioServerPlugIn_IOKitUserClients` so the plugin may open USB devices
  from inside Core Audio's sandbox.

### Protocol notes (USB `0644:8021`)

- Interface 1, alt setting 1. EP `0x81` iso IN (capture), EP `0x02` iso OUT (playback),
  EP `0x83` / `0x04` bulk (MIDI). One packet per 125 µs micro-frame.
- PCM, 24-bit little-endian, 2 interleaved channels.
- Vendor request `bmRequestType 0x40, bRequest 'I'` with `wValue 0x11` then `0x10` to initialise.
- Sample rate: class request `SET_CUR` (`0x22`, `0x01`, `wValue 0x0100`, `wIndex 0x81`, 3 bytes).
- Capture only runs while playback runs; the interface is the clock master.
- An empty input packet means the interface dropped data.
- macOS rejects zero-length iso OUT packets, so the driver never sends one.

The Linux `snd-usb-us122l` driver (`usb_stream`) was a valuable reference for the protocol.

## License

MIT — see `LICENSE`. The driver links [libusb](https://libusb.info) (LGPL-2.1).

---

## Version française

Driver Core Audio open source pour la **TASCAM US-122 MKII** sur **Mac Apple Silicon** (M1 à M4).
Tascam n'a jamais sorti de driver Apple Silicon pour cette carte. Celui-ci tourne entièrement en
espace utilisateur : **pas d'extension noyau, pas de logiciel Tascam, pas de compte développeur
Apple**. Une fois installé, c'est du branche-et-joue : la carte apparaît dans Réglages Système › Son
et dans ton logiciel audio dès qu'elle est branchée.

> Projet non officiel, sans lien avec TASCAM / TEAC.

**État :** lecture et enregistrement stéréo, 44,1 / 48 / 88,2 / 96 kHz, reprise automatique après
la veille et le débranchement. Le MIDI n'est pas encore géré. Latence de sortie ≈ 32 ms.

**Installer :**

```bash
xcode-select --install     # si ce n'est pas déjà fait
brew install libusb
cd ~/Downloads/us122mkii-macos-main
bash build.sh
bash install.sh            # mot de passe administrateur demandé
```

Pour retirer le driver : `bash uninstall.sh`.

**Conseil de niveau :** les potards d'entrée règlent un gain continu ; « LINE » et « MIC » sur la
graduation indiquent seulement la zone conseillée. Vise des crêtes entre −10 et −6 dB dans ton
logiciel plutôt que de remonter la piste ensuite.

Un projet Plage 84.
