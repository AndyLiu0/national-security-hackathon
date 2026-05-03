# =====================================================================
# parts/payload.py — the two main sensor instruments:
#   * EO gimbal (Wescam-style yoke + ball with optical aperture window)
#   * SWIR camera (rectangular housing + cylindrical lens barrel + AR window)
# Each is built as its own sub-component so it shows up cleanly in the
# browser tree under the Sensor_Pod assembly.
# =====================================================================
import adsk.core, adsk.fusion, math
from . import common


# =========================== EO GIMBAL ===============================
def build_eo_gimbal(parent_comp, P, apps,
                    pos_xyz=(0, 0, 0), name='EO_Gimbal'):
    cx, cy, cz = pos_xyz
    occ = common.new_component(parent_comp, name, x=cx, y=cy, z=cz)
    comp = occ.component

    ball_r       = P['eo_ball_r_cm']
    yoke_thick   = P['eo_yoke_thick_cm']
    yoke_arm_w   = P['eo_yoke_arm_w_cm']
    mount_plate  = P['eo_mount_plate_cm']

    # ---- Mount plate (flat box on top of gimbal) ----
    common.make_box(comp,
                    (0, 0, 0),
                    (mount_plate, mount_plate, yoke_thick * 0.6),
                    name='Mount_Plate')
    common.apply_app(comp.bRepBodies.item(comp.bRepBodies.count - 1),
                     apps['metal'])

    # ---- Yoke arms (two vertical pillars cradling the ball) ----
    arm_h = ball_r * 1.5
    for s in (+1, -1):
        common.make_box(comp,
                        (0, s * (ball_r + yoke_arm_w/2),
                         -arm_h/2 - yoke_thick * 0.3),
                        (yoke_arm_w * 1.4, yoke_arm_w, arm_h),
                        name=f'Yoke_Arm_{"R" if s>0 else "L"}')
        common.apply_app(comp.bRepBodies.item(comp.bRepBodies.count - 1),
                         apps['metal'])

    # ---- Ball (the gimbal sphere, full payload housing) ----
    ball_z = -arm_h - yoke_thick * 0.3 + ball_r * 0.3
    ball = common.make_sphere(comp, (0, 0, ball_z), ball_r, name='Gimbal_Ball')
    common.apply_app(ball, apps['pod'])

    # ---- Optical aperture (recessed glass window on chin/forward) ----
    # Forward-looking aperture — small disc protruding from the ball nose.
    apt_r = ball_r * 0.45
    common.make_cylinder(comp,
                         (ball_r * 0.85, 0, ball_z),
                         (ball_r * 0.95, 0, ball_z),
                         apt_r, name='EO_Aperture_Bezel')
    common.apply_app(comp.bRepBodies.item(comp.bRepBodies.count - 1),
                     apps['metal'])
    common.make_cylinder(comp,
                         (ball_r * 0.95, 0, ball_z),
                         (ball_r * 0.99, 0, ball_z),
                         apt_r * 0.85, name='EO_Glass')
    common.apply_app(comp.bRepBodies.item(comp.bRepBodies.count - 1),
                     apps['optic'])

    return occ


# =========================== SWIR CAMERA =============================
def build_swir_camera(parent_comp, P, apps,
                      pos_xyz=(0, 0, 0), name='SWIR_Camera'):
    cx, cy, cz = pos_xyz
    occ = common.new_component(parent_comp, name, x=cx, y=cy, z=cz)
    comp = occ.component

    box_l = P['swir_box_len_cm']
    box_w = P['swir_box_w_cm']
    box_h = P['swir_box_h_cm']
    barrel_r   = P['swir_barrel_r_cm']
    barrel_len = P['swir_barrel_len_cm']

    # ---- Main electronics housing (rectangular box) ----
    common.make_box(comp,
                    (0, 0, 0),
                    (box_l, box_w, box_h),
                    name='SWIR_Housing')
    common.apply_app(comp.bRepBodies.item(comp.bRepBodies.count - 1),
                     apps['metal'])

    # ---- Lens barrel (cylinder protruding forward from front face) ----
    barrel_x_start = box_l / 2
    barrel_x_end   = barrel_x_start + barrel_len
    common.make_cylinder(comp,
                         (barrel_x_start, 0, 0),
                         (barrel_x_end,   0, 0),
                         barrel_r, name='SWIR_Lens_Barrel')
    common.apply_app(comp.bRepBodies.item(comp.bRepBodies.count - 1),
                     apps['metal'])

    # ---- Lens hood (slightly larger ring at the front) ----
    hood_r = barrel_r * 1.18
    common.make_cylinder(comp,
                         (barrel_x_end - 0.6, 0, 0),
                         (barrel_x_end,       0, 0),
                         hood_r, name='SWIR_Hood')
    common.apply_app(comp.bRepBodies.item(comp.bRepBodies.count - 1),
                     apps['pod'])

    # ---- AR-coated window glass ----
    common.make_cylinder(comp,
                         (barrel_x_end - 0.15, 0, 0),
                         (barrel_x_end - 0.05, 0, 0),
                         barrel_r * 0.92, name='SWIR_Window')
    common.apply_app(comp.bRepBodies.item(comp.bRepBodies.count - 1),
                     apps['optic'])

    return occ
