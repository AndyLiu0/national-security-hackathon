# =====================================================================
# parts/fuselage.py - MERGED fuselage + sensor pod.
#
# The fuselage IS the sensor housing. A compact cylindrical pod sits at
# the wing root center, containing all sensors, compute, and avionics.
#
# Sensor layout (COTS Enhanced):
#   TOP:    Flush SWIR InGaAs windows (looking UP at missile thermal sig)
#   BOTTOM: Small EO dome (looking DOWN for visual verification)
#   SIDES:  Flush MEMS infrasound ports (8x, detecting shockwaves)
#   TOP-AFT: 360-degree omnidirectional camera dome
#   INSIDE: Jetson Orin Nano, GPS/INS, LoRa radio, battery
#
# Pod local frame origin = wing root quarter-chord. +X = forward, +Z = up.
# =====================================================================
import adsk.core, adsk.fusion, math
from . import common


def build(parent_comp, P, apps):
    occ = common.new_component(parent_comp, 'Fuselage_Sensor')
    comp = occ.component

    L = P['fus_length_cm']
    R = P['fus_max_r_cm']

    # ============================================================
    # MAIN POD BODY - revolved capsule shape (streamlined cylinder
    # with ogive nose and tapered tail)
    # ============================================================
    sk = comp.sketches.add(comp.xZConstructionPlane)
    pts = [
        (-L * 0.50,          0.0),
        (-L * 0.38,          R * 0.55),
        (-L * 0.25,          R * 0.88),
        (-L * 0.10,          R),
        ( L * 0.10,          R),
        ( L * 0.25,          R * 0.92),
        ( L * 0.38,          R * 0.60),
        ( L * 0.50,          0.0),
    ]
    spts = adsk.core.ObjectCollection.create()
    for (x, z) in pts:
        spts.add(adsk.core.Point3D.create(x, z, 0))
    sk.sketchCurves.sketchFittedSplines.add(spts)
    sk.sketchCurves.sketchLines.addByTwoPoints(
        adsk.core.Point3D.create(-L * 0.50, 0, 0),
        adsk.core.Point3D.create( L * 0.50, 0, 0))

    rev = common.revolve_full(comp, sk.profiles.item(0),
                              comp.xConstructionAxis, 'new')
    body = rev.bodies.item(0)
    body.name = 'Sensor_Pod_Body'
    common.apply_app(body, apps['pod'])

    # ============================================================
    # SWIR WINDOWS - flush transparent panels on TOP (looking UP)
    # Two windows: forward and aft for wider field of regard
    # ============================================================
    sw_len = P.get('swir_window_len_cm', 6.0)
    sw_w   = P.get('swir_window_w_cm', 4.0)
    sw_t   = P.get('swir_window_thick_cm', 0.3)

    for i, x_off in enumerate([L * 0.12, -L * 0.12]):
        common.make_box(
            comp,
            (x_off, 0, R - sw_t / 2 + 0.05),
            (sw_len, sw_w, sw_t),
            name=f'SWIR_Window_{i+1}')
        common.apply_app(comp.bRepBodies.item(comp.bRepBodies.count - 1),
                         apps['optic'])

    # Thin bezel frames around each SWIR window
    bezel_t = 0.15
    for i, x_off in enumerate([L * 0.12, -L * 0.12]):
        common.make_box(
            comp,
            (x_off, 0, R - bezel_t / 2 + 0.08),
            (sw_len + 0.8, sw_w + 0.8, bezel_t),
            name=f'SWIR_Bezel_{i+1}')
        common.apply_app(comp.bRepBodies.item(comp.bRepBodies.count - 1),
                         apps['metal'])

    # ============================================================
    # EO DOME - small protruding dome on BOTTOM (looking DOWN)
    # ============================================================
    eo_r    = P.get('eo_dome_r_cm', 3.0)
    eo_drop = P.get('eo_dome_drop_cm', 1.5)

    eo_ball = common.make_sphere(
        comp,
        (L * 0.10, 0, -R + eo_r * 0.3),
        eo_r,
        name='EO_Dome')
    common.apply_app(eo_ball, apps['optic'])

    # Dome bezel ring
    common.make_cylinder(
        comp,
        (L * 0.10, 0, -R + 0.1),
        (L * 0.10, 0, -R - 0.2),
        eo_r * 1.1,
        name='EO_Bezel')
    common.apply_app(comp.bRepBodies.item(comp.bRepBodies.count - 1),
                     apps['metal'])

    # ============================================================
    # MEMS INFRASOUND PORTS - 8 flush ports around the pod sides
    # Arranged in two rings of 4 (fore and aft) for TDOA processing
    # ============================================================
    mems_r = P.get('mems_port_r_cm', 0.8)
    mems_d = P.get('mems_port_depth_cm', 0.3)

    for ring_x in [L * 0.15, -L * 0.15]:
        for angle_deg in [45, 135, 225, 315]:
            angle = math.radians(angle_deg)
            py = R * 0.95 * math.cos(angle)
            pz = R * 0.95 * math.sin(angle)
            dx, dy, dz_dir = 0, math.cos(angle), math.sin(angle)
            p1 = (ring_x, py, pz)
            p2 = (ring_x,
                  py + dy * mems_d,
                  pz + dz_dir * mems_d)
            common.make_cylinder(comp, p1, p2, mems_r,
                                 name=f'MEMS_Port_{angle_deg}')
            common.apply_app(
                comp.bRepBodies.item(comp.bRepBodies.count - 1),
                apps['metal'])

    # ============================================================
    # 360-DEGREE OMNIDIRECTIONAL CAMERA - single dome on top-aft
    # Cheapest/most robust: single hemispherical lens dome
    # (Ricoh Theta / Insta360-style COTS module, ~$300)
    # Provides full spherical situational awareness + BDA imagery
    # ============================================================
    cam_r = 1.5   # 1.5 cm radius dome (30 mm diameter)
    cam_x = -L * 0.25  # aft of center, clear of SWIR windows
    cam_z = R + cam_r * 0.3  # sits proud on top of pod

    # Glass dome (upper hemisphere)
    common.make_sphere(comp, (cam_x, 0, cam_z), cam_r, name='Cam360_Dome')
    common.apply_app(comp.bRepBodies.item(comp.bRepBodies.count - 1),
                     apps['optic'])

    # Mounting collar
    common.make_cylinder(
        comp,
        (cam_x, 0, cam_z - cam_r * 0.3),
        (cam_x, 0, cam_z - cam_r * 0.3 - 0.4),
        cam_r * 1.2,
        name='Cam360_Mount')
    common.apply_app(comp.bRepBodies.item(comp.bRepBodies.count - 1),
                     apps['metal'])

    # ============================================================
    # COMPUTE BAY PANEL LINE - cosmetic detail showing access panel
    # ============================================================
    common.make_box(
        comp,
        (0, R * 0.98, 0),
        (L * 0.50, 0.15, R * 0.8),
        name='Avionics_Panel_Line')
    common.apply_app(comp.bRepBodies.item(comp.bRepBodies.count - 1),
                     apps['metal'])

    return occ
