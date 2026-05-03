# =====================================================================
# parts/empennage.py — twin tail booms + horizontal stab + vertical fins
# Booms run aft from the wing TE. H-stab spans between the two boom tips.
# A small vertical fin sits on each boom near the tail.
# =====================================================================
import adsk.core, adsk.fusion, math
from . import common


# ---------------------------- BOOMS ----------------------------------
def build_boom(parent_comp, side, P, apps):
    """Single carbon-tube boom extending aft from the wing trailing edge."""
    name = 'Boom_R' if side > 0 else 'Boom_L'

    # Boom origin = wing TE area at this spanwise station, on the wing's
    # mean chord line (slight z lift for dihedral, slight x for sweep)
    half_span = P['wingspan_cm'] / 2.0
    sweep_x_per_y = -math.tan(math.radians(P['sweep_deg']))
    dihed_z_per_y =  math.tan(math.radians(P['dihedral_deg']))
    y_pos = side * P['boom_y_cm']
    x_attach = sweep_x_per_y * abs(y_pos) - P['root_chord_cm'] * 0.10
    z_attach = dihed_z_per_y * abs(y_pos)

    occ = common.new_component(parent_comp, name,
                               x=x_attach, y=y_pos, z=z_attach)
    comp = occ.component

    # Build cylinder along -X (aft)
    common.make_cylinder(
        comp,
        (0.0, 0.0, 0.0),
        (-P['boom_length_cm'], 0.0, 0.0),
        P['boom_radius_cm'],
        name=name + '_Tube')
    common.apply_app(comp.bRepBodies.item(0), apps['carbon'])

    # Slight nose cap (cone) at the front for aero clean-up
    common.make_cone(
        comp,
        (0.0, 0.0, 0.0),    P['boom_radius_cm'],
        (3.0, 0.0, 0.0),    0.05,
        name=name + '_NoseCap')
    common.apply_app(comp.bRepBodies.item(comp.bRepBodies.count - 1),
                     apps['carbon'])

    return occ


# --------------------------- H-STAB ----------------------------------
def build_htail(parent_comp, P, apps):
    """Symmetric airfoil H-stab spanning between the two boom tips."""
    half_span = P['wingspan_cm'] / 2.0
    sweep_x_per_y = -math.tan(math.radians(P['sweep_deg']))
    dihed_z_per_y =  math.tan(math.radians(P['dihedral_deg']))
    boom_attach_x = sweep_x_per_y * P['boom_y_cm'] - P['root_chord_cm'] * 0.10
    boom_z = dihed_z_per_y * P['boom_y_cm']
    htail_x = boom_attach_x - P['boom_length_cm'] + P['htail_root_cm'] * 0.5

    occ = common.new_component(parent_comp, 'HStab',
                               x=htail_x, y=0.0, z=boom_z)
    comp = occ.component

    # Center (Y=0) profile — fewer points for small chords
    centerSk = comp.sketches.add(comp.xZConstructionPlane)
    npts_h = 20 if P['htail_root_cm'] < 40 else 40
    raw = common.naca4(0, 0, 9, P['htail_root_cm'], n=npts_h)
    qc = P['htail_root_cm'] * 0.25
    pts = [(qc - x, z) for (x, z) in raw]
    common.add_closed_spline(centerSk, pts)

    # Loft to each tip (one solid per side, joined)
    for s in (+1, -1):
        tipPlane = common.offset_plane(comp, comp.xZConstructionPlane,
                                       s * P['boom_y_cm'])
        tipSk = comp.sketches.add(tipPlane)
        raw_t = common.naca4(0, 0, 9, P['htail_tip_cm'], n=npts_h)
        qct = P['htail_tip_cm'] * 0.25
        tip_pts = [(qct - x, z) for (x, z) in raw_t]
        common.add_closed_spline(tipSk, tip_pts)

        op = 'new' if s > 0 else 'join'
        common.loft_solid(comp,
                          [centerSk.profiles.item(0),
                           tipSk.profiles.item(0)], op)

    for b in comp.bRepBodies:
        common.apply_app(b, apps['skin'])
    return occ


# -------------------------- VERTICAL FIN -----------------------------
def build_vfin(parent_comp, side, P, apps):
    """Vertical stabilizer mounted atop each boom near the tail."""
    name = 'VFin_R' if side > 0 else 'VFin_L'

    half_span = P['wingspan_cm'] / 2.0
    sweep_x_per_y = -math.tan(math.radians(P['sweep_deg']))
    dihed_z_per_y =  math.tan(math.radians(P['dihedral_deg']))
    y_pos = side * P['boom_y_cm']
    boom_attach_x = sweep_x_per_y * abs(y_pos) - P['root_chord_cm'] * 0.10
    boom_z = dihed_z_per_y * abs(y_pos)
    fin_x = boom_attach_x - P['boom_length_cm'] + P['vtail_chord_cm'] * 0.5

    occ = common.new_component(parent_comp, name,
                               x=fin_x, y=y_pos, z=boom_z + P['boom_radius_cm'])
    comp = occ.component

    # Lofted symmetric airfoil from boom (root) to top (tip)
    h = P['vtail_height_cm']
    cr = P['vtail_chord_cm']
    ct = cr * 0.55
    sweep_x_fin = -h * math.tan(math.radians(15))  # 15° aft sweep for the fin

    # Root profile in XY plane (Z=0 at attachment) — fewer pts for small chords
    rootSk = comp.sketches.add(comp.xYConstructionPlane)
    npts_v = 20 if cr < 40 else 40
    raw_r = common.naca4(0, 0, 9, cr, n=npts_v)
    qcr = cr * 0.25
    common.add_closed_spline(rootSk, [(qcr - x, z) for (x, z) in raw_r])

    # Tip plane offset along Z by h
    tipPlane = common.offset_plane(comp, comp.xYConstructionPlane, h)
    tipSk = comp.sketches.add(tipPlane)
    raw_t = common.naca4(0, 0, 9, ct, n=npts_v)
    qct = ct * 0.25
    tip_pts = [(qct - x + sweep_x_fin, z) for (x, z) in raw_t]
    common.add_closed_spline(tipSk, tip_pts)

    common.loft_solid(comp,
                      [rootSk.profiles.item(0),
                       tipSk.profiles.item(0)], 'new')

    for b in comp.bRepBodies:
        common.apply_app(b, apps['skin'])
    return occ
