# ARGUS Airframe — CAD

Parametric Fusion 360 assembly script for a Zephyr-inspired HAPS solar glider, optimized for the COTS Enhanced configuration (8.0 m span, 11.5 kg MTOW).

## Running

1. Copy `argus_airframe/` into your Fusion 360 Scripts directory
2. Utilities > ADD-INS > Scripts and Add-ins > My Scripts > argus_airframe > Run
3. The script generates a full assembly with sub-components in the browser tree

## Design Choices

| Requirement | CAD Feature |
|---|---|
| Multi-day stratospheric endurance | High-AR wing (26.7), full-span solar array on upper surface |
| Passive sensing / low self-noise | Merged fuselage-sensor pod with flush-mount sensors, tractor props forward of wing |
| Upward-looking missile detection | SWIR InGaAs windows flush on pod top, looking UP against cold stratospheric background |
| Visual verification | EO glass dome on pod bottom, looking DOWN |
| 360 degree situational awareness | Omnidirectional camera dome (top-aft of pod) |
| Long-baseline TDOA infrasound | MEMS mic ports at wingtips + boom tails (max baseline) |
| Differential pressure / airspeed | Nose Pitot probe + fin-mounted pressure ports |

## Key Parameters (top of `argus_airframe.py`)

All dimensions in centimeters, angles in degrees. Edit and re-run to regenerate.

## Legacy Files

- `ARGUS_Airframe v2.stl` — earlier OpenSCAD mesh export (superseded by Fusion script)
- `argus_airframe/parts/sensor_pod.py` — standalone sensor pod (now merged into fuselage.py)
- `argus_airframe/parts/payload.py` — standalone EO/SWIR builders (now integrated into fuselage)
