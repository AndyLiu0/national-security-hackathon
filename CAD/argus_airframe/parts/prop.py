# =====================================================================
# parts/prop.py - 4-blade tractor propeller with spinner + hub.
# Each blade is lofted through 7 airfoil stations with chord taper and
# aerodynamic twist. The loft uses guide rails (leading edge + trailing
# edge splines through all stations) so the surface connects cleanly.
#
# Local frame: prop axis = +X (forward). Hub at X=0.
# =====================================================================
import adsk.core, adsk.fusion, math
from . import common


# Blade stations: (r/R, chord_frac_of_radius, twist_deg)
_BLADE_STATIONS = [
    (0.18,  0.18,  35),    # near hub - high pitch
    (0.35,  0.22,  28),
    (0.50,  0.22,  22),
    (0.65,  0.20,  17),
    (0.80,  0.16,  13),
    (0.92,  0.11,  10),
    (1.00,  0.06,   7),    # tip - low pitch
]


def build(parent_comp, P, apps, name='Propeller', x=0.0, y=0.0, z=0.0):
    """Build a 4-blade propeller as a sub-component at (x, y, z)."""
    occ = common.new_component(parent_comp, name, x=x, y=y, z=z)
    comp = occ.component

    radius      = P['prop_diam_cm'] / 2.0
    hub_r       = P['prop_hub_radius_cm']
    spinner_len = P['prop_spinner_len_cm']

    # ---------------- Spinner (forward of hub) ----------------
    common.make_cone(
        comp,
        (0.0, 0.0, 0.0),  hub_r * 0.95,
        (spinner_len, 0.0, 0.0), 0.05,
        name='Spinner')
    common.apply_app(comp.bRepBodies.item(comp.bRepBodies.count - 1),
                     apps['spinner'])

    # ---------------- Hub (cylinder behind spinner) -----------
    common.make_cylinder(
        comp,
        (-P['prop_hub_len_cm'], 0.0, 0.0),
        (0.0, 0.0, 0.0),
        hub_r,
        name='Hub')
    common.apply_app(comp.bRepBodies.item(comp.bRepBodies.count - 1),
                     apps['spinner'])

    # ---------------- Blade (lofted through stations) ----------
    profiles = []
    le_points = []
    te_points = []

    for (r_norm, c_frac, twist_deg) in _BLADE_STATIONS:
        r = r_norm * radius
        chord = c_frac * radius

        plane = common.offset_plane(comp, comp.xZConstructionPlane, r)
        sk = comp.sketches.add(plane)

        npts = 16 if chord < 4 else 20
        raw = common.naca4(4, 4, 12, chord, n=npts, te_thick_pct=1.5)
        qc = chord * 0.25
        pts_local = [(qc - x_, z_) for (x_, z_) in raw]

        ct = math.cos(math.radians(twist_deg))
        st = math.sin(math.radians(twist_deg))
        pts_twist = [(u*ct - v*st, u*st + v*ct) for (u, v) in pts_local]

        common.add_closed_spline(sk, pts_twist)
        profiles.append(sk.profiles.item(0))

        le_u, le_v = max(pts_twist, key=lambda p: p[0])
        te_u, te_v = min(pts_twist, key=lambda p: p[0])

        le_points.append(adsk.core.Point3D.create(le_u, r, le_v))
        te_points.append(adsk.core.Point3D.create(te_u, r, te_v))

    # Build guide rail splines
    railSk = comp.sketches.add(comp.xYConstructionPlane)

    le_col = adsk.core.ObjectCollection.create()
    for p in le_points:
        le_col.add(p)
    le_rail = railSk.sketchCurves.sketchFittedSplines.add(le_col)

    te_col = adsk.core.ObjectCollection.create()
    for p in te_points:
        te_col.add(p)
    te_rail = railSk.sketchCurves.sketchFittedSplines.add(te_col)

    # Loft with guide rails
    feats = comp.features.loftFeatures
    li = feats.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    for prof in profiles:
        li.loftSections.add(prof)
    li.isSolid = True

    try:
        li.centerLineOrRails.addRail(le_rail)
        li.centerLineOrRails.addRail(te_rail)
    except:
        pass

    try:
        blade_loft = feats.add(li)
        blade_body = blade_loft.bodies.item(0)
    except:
        blade_loft = common.loft_solid(comp, profiles, 'new')
        blade_body = blade_loft.bodies.item(0)

    blade_body.name = 'Blade_1'
    common.apply_app(blade_body, apps['carbon'])

    # ---------------- Circular pattern: 4 blades around X axis --------
    try:
        bodies_to_pattern = adsk.core.ObjectCollection.create()
        bodies_to_pattern.add(blade_body)
        cpFeats = comp.features.circularPatternFeatures
        cpInput = cpFeats.createInput(bodies_to_pattern,
                                       comp.xConstructionAxis)
        cpInput.quantity = adsk.core.ValueInput.createByReal(4)
        cpInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        cpInput.isSymmetric = False
        cpFeats.add(cpInput)
    except:
        for i, angle_deg in enumerate([90, 180, 270]):
            try:
                angle = math.radians(angle_deg)
                mFeats = comp.features.moveFeatures
                bodies_coll = adsk.core.ObjectCollection.create()
                bodies_coll.add(blade_body)
                mi = mFeats.createInput2(bodies_coll)
                rot = adsk.core.Matrix3D.create()
                rot.setToRotation(angle,
                                  adsk.core.Vector3D.create(1, 0, 0),
                                  adsk.core.Point3D.create(0, 0, 0))
                mi.defineAsTransform(rot)
                mi.isCopy = True
                feat = mFeats.add(mi)
                for b in feat.bodies:
                    b.name = f'Blade_{i+2}'
                    common.apply_app(b, apps['carbon'])
            except:
                pass

    return occ
