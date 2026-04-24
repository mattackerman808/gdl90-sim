# GDL90 Flight Sim

Micro flight simulator that drives Tachbase's GDL90 receiver plugin over UDP.
Fly a little airplane around Northern California with arrow keys; the iOS app
sees synthetic ownship + altitude + heartbeat packets at 1–2 Hz.

## Run

```bash
python3 server.py
# then open http://127.0.0.1:8765
```

Defaults emit UDP to `127.0.0.1:4000` (the iOS Simulator, which forwards
127.0.0.1 to the Mac host). For a physical iPad, override the UDP target:

```bash
python3 server.py --target 192.168.1.42:4000
```

or change it live in the toolbar `UDP` field.

## Controls

| Key                    | Action                                       |
| ---------------------- | -------------------------------------------- |
| `Space`                | Start / Stop transmitting                    |
| `←` `→`                | Turn at 3°/sec (Shift = 6°/sec)              |
| `↑` `↓`                | Climb/descend at 500 fpm (Shift = 1000 fpm)  |
| `+` `−`                | Ground speed ±10 kt                          |
| `R`                    | Reset to selected start airport              |
| `T` then click         | Teleport ownship to clicked point            |

## What it emits (Phase A)

- **0x00** Heartbeat — 1 Hz, only while `/pos` has been fresh within 3 s
- **0x0A** Ownship Report — 2 Hz (lat/lon/alt/track/gs/vs)
- **0x0B** Ownship Geometric Altitude — 2 Hz

## Roadmap (from the plan)

- **B**: Traffic reports (0x14) with scripted targets
- **C**: AHRS via ForeFlight extended 0x65 subID 1
- **D**: Load a Tachbase flight plan and autopilot it at a set GS
- **E**: Scripted scenarios (hold entry, DME arc, missed approach)

## No deps

Pure Python stdlib on the server side; MapLibre GL JS loaded from a CDN on
the browser side. Map tiles come from `cdn.tachbase.com` for 1:1 visual
match with the iOS app — the AIRAC cycle is auto-detected via
`/cgi-bin/tile_manifest.py`.
