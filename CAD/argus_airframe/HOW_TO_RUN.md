# Running the ARGUS Fusion 360 Script

This script generates the full ARGUS airframe as a proper Fusion **assembly** — every part (wings, fuselage, booms, tails, nacelles, sensor pod, mics) is its own named component in the browser tree, with engineering-grade appearances applied.

You only need to do this **once**. After it runs you have a normal `.f3d` file you can edit interactively, render, export to STEP, etc.

## One-time install (≈2 minutes)

1. Open **Fusion 360**.
2. Top toolbar → **Utilities** → **ADD-INS** → **Scripts and Add-ins…** (shortcut: `Shift + S`).
3. In the dialog, make sure the **Scripts** tab (not Add-Ins) is selected.
4. Next to **My Scripts** click the green **+** icon → **Script from local folder**.
   - *(On older Fusion versions: click the small **+** beside "My Scripts" then browse.)*
5. Navigate to and select this folder:
   ```
   CAD/argus_airframe/
   ```
   (The folder containing both `argus_airframe.py` and `argus_airframe.manifest`.)
6. **ARGUS Airframe** now appears under My Scripts. Select it → click **Run**.

That's it. A new untitled design opens and the airframe builds itself in ~10–30 seconds. You'll see a confirmation dialog when it's done.

> **If "Script from local folder" isn't an option** in your Fusion version: instead, copy the `argus_airframe/` folder into Fusion's default scripts directory and it'll appear automatically next time you open the dialog:
> - **Windows**: `%APPDATA%\Autodesk\Autodesk Fusion 360\API\Scripts\`
> - **Mac**: `~/Library/Application Support/Autodesk/Autodesk Fusion 360/API/Scripts/`

## After it runs

In the browser tree (left panel) you'll see:
```
ARGUS_Airframe
├── Fuselage
├── Wing_R          ← solar PV faces painted directly on upper surface
├── Wing_L              (fixes the offset-panel bug — they can't lift off)
├── Boom_R
├── Boom_L
├── HStab
├── VFin_R
├── VFin_L
├── Nacelle_R       ← geometry intersects wing underside (not floating)
├── Nacelle_L
├── SensorPod
│   ├── pod body
│   ├── SWIR window (glass)
│   ├── EO gimbal turret (metal)
│   └── pylon (white skin)
└── InfrasoundArray (5 mics: 2 wingtips, 2 boom tails, 1 nose)
```

### To get a hyper-realistic render
1. Switch workspace (top-left dropdown): **Design → Render**.
2. **Scene Settings** (right panel) → choose environment **Sharp Highlights** or **Cool Light**, set **Background → Solid color (light grey)**.
3. **In-canvas Render** (the "play" icon) for an interactive preview, or **Render** with **Cloud rendering** + Excellent quality for a publishable still.

### To save / share
- **File → Export → .f3d** for a portable Fusion archive.
- **File → Export → STEP (.stp)** for handing to anyone using SolidWorks/Onshape/etc.
- **File → 3D Print → STL** for the physical prototype phase.

## Tweaking the design

All dimensions live in the `P = { ... }` dictionary at the top of `argus_airframe.py`. Common tweaks:

| Want to change… | Edit |
|---|---|
| Wingspan | `wingspan_cm` (default 2500 = 25 m) |
| Aspect ratio | `root_chord_cm`, `tip_chord_cm` |
| Sensor isolation distance | `pod_drop_cm`, `pod_forward_cm` |
| TDOA mic baseline | `boom_y_cm`, `boom_length_cm` |
| Bench-prototype scale | divide all `*_cm` by 50 (or globally scale via search/replace) |

After editing, re-run the script — it creates a *new* document each time so your previous version is preserved.

## What's different from the OpenSCAD version

| Bug you flagged | Fix |
|---|---|
| Nacelles weren't attached to wings | Nacelle position now computed from wing's local chord, sweep, and dihedral at the nacelle's spanwise station — geometry physically intersects the wing skin so it can never appear detached |
| Solar panels drifted off the wing at the tip | PV is no longer a separate body. Each upward-facing face of the lofted wing is **face-painted** with a dark blue PV appearance, so the "panel" is the wing surface itself and follows dihedral/washout exactly |

## Known limitations / nice-to-haves

- **No real prop blades** — the prop disc is a thin cylinder standing in for motion blur. If you want modeled blades, ask and I'll add a 2- or 3-blade variable-pitch generator.
- **No assembly joints** — components are positioned correctly but not constrained with Fusion *joints*. This is fine for rendering and engineering reference; if you want articulating control surfaces or a steerable EO gimbal, joints can be added in a follow-up.
- **NACA 4412 airfoil** — chosen for code simplicity. For a more credibly HALE-appropriate section (SD7037, FX 63-137), swap the `naca4_pts()` call for an embedded coordinate table.
