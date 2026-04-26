# GDL90 Flight Sim

Micro flight simulator that drives Tachbase's GDL90 receiver plugin over UDP.
Fly a little airplane around Northern California with arrow keys; the iOS app
sees synthetic ownship + altitude + heartbeat packets at 1–2 Hz.

## Run

```bash
python3 server.py
# then open http://127.0.0.1:8765
```

By default the sim broadcasts on the Mac's Wi-Fi subnet (e.g. `192.168.1.255:4000`),
mirroring how a real Stratus emits packets on its own AP. Any iPad on the
same Wi-Fi running Tachbase will pick it up — no IP lookup needed.

Overrides:

```bash
python3 server.py --target 127.0.0.1:4000        # iOS Simulator
python3 server.py --target 192.168.1.42:4000     # specific iPad
python3 server.py --target 255.255.255.255:4000  # limited broadcast
```

You can also change the target live in the toolbar `UDP` field.

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

## Deps

Optional but strongly recommended:

```bash
pip install pygeomag
```

Without `pygeomag` the sim falls back to a one-line linear approximation
of magnetic declination, which produces a permanent ~3° crab in the HSI
track diamond on the iPad even with zero wind. With `pygeomag` installed
the sim emits physically-consistent ownship-true and AHRS-magnetic values
(matching what real Stratus hardware does), and the diamond sits cleanly
at the lubber.

Everything else is Python stdlib. MapLibre GL JS loads from a CDN on the
browser side, and map tiles come from `cdn.tachbase.com` for 1:1 visual
match with the iOS app — the AIRAC cycle is auto-detected via
`/cgi-bin/tile_manifest.py`.
