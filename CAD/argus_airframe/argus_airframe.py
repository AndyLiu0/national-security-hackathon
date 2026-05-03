# =====================================================================
# Project ARGUS — Stratospheric Solar-Glider Airframe (Master Script)
#
# Builds a full-scale (25 m wingspan) Zephyr-style HALE platform as a
# proper Fusion 360 assembly with sub-assemblies for each major part.
# Each part lives in its own module under parts/.
#
# Run from: Utilities -> ADD-INS -> Scripts and Add-ins -> My Scripts
#           -> ARGUS Airframe -> Run
# =====================================================================
import adsk.core, adsk.fusion, traceback, os, sys, importlib

# Make our parts/ subpackage importable regardless of how Fusion launches us
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

# Force reload of all parts modules so edits take effect on re-run
_PARTS_MODULES = ['parts',
                  'parts.common', 'parts.wing', 'parts.solar_array',
                  'parts.fuselage', 'parts.empennage', 'parts.prop',
                  'parts.nacelle', 'parts.payload', 'parts.fittings',
                  'parts.sensor_pod']
for _m in _PARTS_MODULES:
    if _m in sys.modules:
        try:
            importlib.reload(sys.modules[_m])
        except:
            pass

from parts import (common, wing, fuselage, empennage, nacelle, fittings)


# =====================================================================
# DESIGN PARAMETERS — tweak any of these and re-run to regenerate
# All distances in CENTIMETERS, all angles in DEGREES.
# =====================================================================
P = {
    # ----- Airfoil (NACA 4-digit) -----
    'airfoil_camber_pct':   4,     # 4% camber
    'airfoil_camber_pos':   4,     # max camber at 40% chord
    'airfoil_thick_pct':   12,     # 12% thick

    # ----- Wing (COTS Enhanced optimized: 8.0 m span, 0.30 m chord) -----
    'wingspan_cm':        800,     # 8.0 m — optimized for COTS Enhanced
    'root_chord_cm':       34,     # 34 cm root (slight taper from 30 cm mean)
    'tip_chord_cm':        26,     # 26 cm tip — AR ~26.7
    'sweep_deg':            3,     # mild sweep for CG
    'dihedral_deg':         4,     # extra dihedral for roll stability
    'washout_deg':          1.5,   # mild washout

    # ----- Solar array (smaller cells for 30 cm chord) -----
    'solar_cell_cm':          5.0,    # 50 mm mini GaAs cells
    'solar_cell_gap_cm':      0.3,
    'solar_inset_cm':         3.0,    # margin from LE/TE
    'solar_inset_root_cm':   10.0,    # margin from fuselage
    'solar_inset_tip_cm':     8.0,    # margin from wingtip
    'solar_tile_thickness_cm': 0.08,  # 0.8 mm thin-film

    # ----- Fuselage / Sensor Pod (MERGED — fuselage IS the sensor pod) -----
    # Cylindrical pod housing all sensors, compute, and avionics
    'fus_length_cm':       45,     # 45 cm total pod length
    'fus_max_r_cm':         7,     # 7 cm radius (14 cm diameter)

    # ----- Sensor windows on fuselage -----
    # SWIR flush windows on top (looking UP at missiles)
    'swir_window_len_cm':    6.0,
    'swir_window_w_cm':      4.0,
    'swir_window_thick_cm':  0.3,
    # EO dome on bottom (looking DOWN for verification)
    'eo_dome_r_cm':          3.0,
    'eo_dome_drop_cm':       1.5,
    # Infrasound MEMS ports (flush-mount on pod sides)
    'mems_port_r_cm':        0.8,
    'mems_port_depth_cm':    0.3,

    # ----- Empennage (scaled for 8 m span) -----
    'boom_length_cm':      70,     # shorter booms
    'boom_radius_cm':       1.2,   # thin CF tubes
    'boom_y_cm':           80,     # boom spanwise station
    'htail_root_cm':       22,
    'htail_tip_cm':        16,
    'vtail_height_cm':     22,
    'vtail_chord_cm':      18,

    # ----- Propulsion (scaled for 11.5 kg MTOW) -----
    'nacelle_y_cm':       200,     # nacelle spanwise station
    'nacelle_len_cm':      18,     # compact nacelle
    'nacelle_r_cm':         2.5,   # small motor pod
    'nacelle_drop_cm':      5,     # pylon drop below wing
    'nacelle_pylon_chord_cm': 8,
    'nacelle_pylon_thick_cm': 1.2,
    'prop_diam_cm':        50,     # 50 cm (20 in) slow prop
    'prop_hub_radius_cm':   2.0,
    'prop_hub_len_cm':      1.5,
    'prop_spinner_len_cm':  4.5,

    # ----- Microphones (MEMS infrasound — flush mount) -----
    'mic_collar_r_cm':      0.6,
    'mic_collar_h_cm':      0.4,
    'mic_foam_r_cm':        0.9,
    'mic_foam_h_cm':        1.5,

    # ----- Pressure ports (Pitot probes) -----
    'pport_base_r_cm':      0.5,
    'pport_base_h_cm':      0.3,
    'pport_tube_r_cm':      0.25,
    'pport_tube_h_cm':      2.5,

    # ----- GPS/Comms antenna (top of fuselage) -----
    'satcom_plate_r_cm':    2.5,
    'satcom_plate_h_cm':    0.3,
    'satcom_dome_r_cm':     2.0,

    # Legacy keys kept for any modules that reference them (unused but prevents KeyError)
    'eo_ball_r_cm':         3.0,
    'eo_yoke_thick_cm':     0.8,
    'eo_yoke_arm_w_cm':     1.0,
    'eo_mount_plate_cm':    4.0,
    'swir_box_len_cm':      6.0,
    'swir_box_w_cm':        4.0,
    'swir_box_h_cm':        3.5,
    'swir_barrel_r_cm':     1.5,
    'swir_barrel_len_cm':   3.0,
    'pod_length_cm':       45,
    'pod_radius_cm':        7,
    'pod_drop_cm':          0,
    'pod_forward_cm':       0,
    'pod_pylon_chord_cm':   8,
    'pod_pylon_thick_cm':   1.2,
}


# =====================================================================
# BUILDERS — each returns the new Occurrence so it can be referenced
# later. We wrap each call in try/except so a single part failure
# doesn't kill the whole assembly build (you'll get a partial assembly
# plus a list of failures in the final dialog).
# =====================================================================
def _safe(label, fn, failures):
    try:
        return fn()
    except Exception as e:
        failures.append(f'{label}: {e}')
        return None


def run(context):
    ui = None
    failures = []
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface

        # --- New document ---
        doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        try:
            doc.name = 'ARGUS_Airframe'
        except:
            pass

        # Direct modeling = faster builds + no timeline bloat for a
        # generated assembly with hundreds of features.
        try:
            design.designType = adsk.fusion.DesignTypes.DirectDesignType
        except:
            pass

        try:
            design.unitsManager.defaultLengthUnits = 'm'
        except:
            pass

        root = design.rootComponent

        # --- Appearance palette (single source of truth) ---
        apps = common.build_appearance_palette(app, design)

        # --- Build the assembly, part by part as sub-assemblies ---
        # Fuselage IS the sensor pod — merged design
        _safe('Fuselage_Sensor', lambda: fuselage.build(root, P, apps), failures)
        _safe('Wing_R',       lambda: wing.build(root, +1, P, apps), failures)
        _safe('Wing_L',       lambda: wing.build(root, -1, P, apps), failures)
        _safe('Boom_R',       lambda: empennage.build_boom(root, +1, P, apps), failures)
        _safe('Boom_L',       lambda: empennage.build_boom(root, -1, P, apps), failures)
        _safe('HStab',        lambda: empennage.build_htail(root, P, apps), failures)
        _safe('VFin_R',       lambda: empennage.build_vfin(root, +1, P, apps), failures)
        _safe('VFin_L',       lambda: empennage.build_vfin(root, -1, P, apps), failures)
        _safe('Nacelle_R',    lambda: nacelle.build(root, +1, P, apps), failures)
        _safe('Nacelle_L',    lambda: nacelle.build(root, -1, P, apps), failures)
        # GPS/Comms antenna on top of fuselage pod
        _safe('Comms_Antenna',
              lambda: fittings.build_satcom(
                  root, P, apps,
                  pos_xyz=(P['fus_length_cm'] * 0.15,
                           0,
                           P['fus_max_r_cm'] - 0.3)),
              failures)
        # Wingtip MEMS infrasound mics (long-baseline TDOA)
        half = P['wingspan_cm'] / 2.0
        import math as _m
        sx = -half * _m.tan(_m.radians(P['sweep_deg']))
        dz =  half * _m.tan(_m.radians(P['dihedral_deg']))
        for s, lbl in ((+1, 'R'), (-1, 'L')):
            _safe(f'Wingtip_Mic_{lbl}',
                  lambda s=s, lbl=lbl: fittings.build_mic(
                      root, P, apps,
                      pos_xyz=(sx, s*half, dz),
                      axis='+z',
                      name=f'Wingtip_Mic_{lbl}'),
                  failures)
        # Boom-tail MEMS mics (long-baseline TDOA)
        boom_attach_x = sx - P['root_chord_cm'] * 0.10
        for s, lbl in ((+1, 'R'), (-1, 'L')):
            _safe(f'Boom_Mic_{lbl}',
                  lambda s=s, lbl=lbl: fittings.build_mic(
                      root, P, apps,
                      pos_xyz=(boom_attach_x - P['boom_length_cm'] + 0.5,
                               s * P['boom_y_cm'],
                               dz),
                      axis='-x',
                      name=f'Boom_Mic_{lbl}'),
                  failures)


        # Pressure ports on vertical fins (differential pressure / AoA sensing)
        # Each fin gets a forward-facing Pitot probe at mid-height on the
        # leading edge, sitting flush against the fin surface.
        sweep_x_per_y = -_m.tan(_m.radians(P['sweep_deg']))
        dihed_z_per_y = _m.tan(_m.radians(P['dihedral_deg']))
        fin_half_chord = P['vtail_chord_cm'] * 0.5
        fin_half_h = P['vtail_height_cm'] * 0.5
        fin_sweep_x = -fin_half_h * _m.tan(_m.radians(15))  # 15 deg fin sweep
        for s, lbl in ((+1, 'R'), (-1, 'L')):
            fin_y = s * P['boom_y_cm']
            fin_base_x = (sweep_x_per_y * abs(fin_y)
                          - P['root_chord_cm'] * 0.10
                          - P['boom_length_cm']
                          + P['vtail_chord_cm'] * 0.5)
            fin_base_z = dihed_z_per_y * abs(fin_y) + P['boom_radius_cm']
            # Probe at mid-height on the LE of the fin
            probe_x = fin_base_x + fin_half_chord + fin_sweep_x * 0.5
            probe_z = fin_base_z + fin_half_h
            _safe(f'Fin_PPort_{lbl}',
                  lambda s=s, lbl=lbl, px=probe_x, pz=probe_z, fy=fin_y:
                      fittings.build_pressure_port(
                          root, P, apps,
                          pos_xyz=(px, fy, pz),
                          axis='+x',
                          name=f'Fin_PressurePort_{lbl}'),
                  failures)

        # Nose Pitot probe - differential pressure (airspeed) sensor
        # Mounted at the very front of the fuselage pod
        _safe('Nose_Pitot',
              lambda: fittings.build_pressure_port(
                  root, P, apps,
                  pos_xyz=(P['fus_length_cm'] * 0.50 + 0.5, 0, 0),
                  axis='+x',
                  name='Nose_Pitot'),
              failures)

        # --- Frame the view ---
        try:
            vp = app.activeViewport
            vp.fit()
        except:
            pass

        # --- Done ---
        if failures:
            ui.messageBox(
                'ARGUS airframe built with some part failures:\n\n' +
                '\n'.join(failures) +
                '\n\nThe rest of the assembly is intact in the browser.')
        else:
            ui.messageBox(
                'ARGUS airframe built successfully.\n\n'
                'All parts are sub-assemblies in the browser tree.\n'
                'Switch to the Render workspace and apply Scene Settings\n'
                '-> Sharp Highlights for a clean look.')
    except:
        if ui:
            ui.messageBox('Script failed:\n{}'.format(traceback.format_exc()))
