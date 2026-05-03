# =====================================================================
# parts/fittings.py — small repeated detail components:
#   * Microphone unit (foam windscreen + collar)
#   * Pressure port (Pitot-style probe)
#   * SatCom radome (low-profile dome)
# =====================================================================
import adsk.core, adsk.fusion, math
from . import common


# ============================ MICROPHONE =============================
def build_mic(parent_comp, P, apps, pos_xyz, axis='+z', name='Mic'):
    """Mic unit: a metal collar mount with a foam windscreen on top.
    `axis` ('+x','-x','+y','-y','+z','-z') controls which way the windscreen
    points (away from the mounting surface)."""
    cx, cy, cz = pos_xyz
    occ = common.new_component(parent_comp, name, x=cx, y=cy, z=cz)
    comp = occ.component

    collar_r  = P['mic_collar_r_cm']
    collar_h  = P['mic_collar_h_cm']
    foam_r    = P['mic_foam_r_cm']
    foam_h    = P['mic_foam_h_cm']

    # Direction vector
    dirs = {'+x': (1,0,0), '-x': (-1,0,0),
            '+y': (0,1,0), '-y': (0,-1,0),
            '+z': (0,0,1), '-z': (0,0,-1)}
    dx, dy, dz = dirs.get(axis, (0,0,1))

    # Collar (metal cylinder flush with mounting surface)
    p1 = (0, 0, 0)
    p2 = (dx*collar_h, dy*collar_h, dz*collar_h)
    common.make_cylinder(comp, p1, p2, collar_r, name='Collar')
    common.apply_app(comp.bRepBodies.item(comp.bRepBodies.count - 1),
                     apps['metal'])

    # Foam windscreen (textured cylinder on top of collar)
    p3 = (dx*(collar_h + foam_h), dy*(collar_h + foam_h),
          dz*(collar_h + foam_h))
    common.make_cylinder(comp, p2, p3, foam_r, name='Windscreen')
    common.apply_app(comp.bRepBodies.item(comp.bRepBodies.count - 1),
                     apps['foam'])

    return occ


# ============================ PRESSURE PORT ==========================
def build_pressure_port(parent_comp, P, apps, pos_xyz,
                        axis='+x', name='PressurePort'):
    """Pitot-style flush-mount pressure port."""
    cx, cy, cz = pos_xyz
    occ = common.new_component(parent_comp, name, x=cx, y=cy, z=cz)
    comp = occ.component

    base_r  = P['pport_base_r_cm']
    base_h  = P['pport_base_h_cm']
    tube_r  = P['pport_tube_r_cm']
    tube_h  = P['pport_tube_h_cm']

    dirs = {'+x': (1,0,0), '-x': (-1,0,0),
            '+y': (0,1,0), '-y': (0,-1,0),
            '+z': (0,0,1), '-z': (0,0,-1)}
    dx, dy, dz = dirs.get(axis, (1,0,0))

    # Mounting flange
    p0 = (0, 0, 0)
    p1 = (dx*base_h, dy*base_h, dz*base_h)
    common.make_cylinder(comp, p0, p1, base_r, name='Flange')
    common.apply_app(comp.bRepBodies.item(comp.bRepBodies.count - 1),
                     apps['metal'])

    # Probe tube
    p2 = (dx*(base_h + tube_h), dy*(base_h + tube_h),
          dz*(base_h + tube_h))
    common.make_cylinder(comp, p1, p2, tube_r, name='Probe')
    common.apply_app(comp.bRepBodies.item(comp.bRepBodies.count - 1),
                     apps['metal'])

    return occ


# ============================ SATCOM ANTENNA =========================
def build_satcom(parent_comp, P, apps, pos_xyz, name='SatCom_Antenna'):
    """Low-profile SatCom radome: hemispherical dome on a thin plate."""
    cx, cy, cz = pos_xyz
    occ = common.new_component(parent_comp, name, x=cx, y=cy, z=cz)
    comp = occ.component

    plate_r = P['satcom_plate_r_cm']
    plate_h = P['satcom_plate_h_cm']
    dome_r  = P['satcom_dome_r_cm']

    # Mounting plate
    common.make_cylinder(comp,
                         (0, 0, 0),
                         (0, 0, plate_h),
                         plate_r, name='Plate')
    common.apply_app(comp.bRepBodies.item(comp.bRepBodies.count - 1),
                     apps['metal'])

    # Hemispherical dome — build full sphere centered at plate top, then
    # we leave it as a sphere (the lower half buries into the plate).
    common.make_sphere(comp,
                       (0, 0, plate_h),
                       dome_r, name='Radome')
    common.apply_app(comp.bRepBodies.item(comp.bRepBodies.count - 1),
                     apps['radome'])

    return occ
