# =====================================================================
# parts/nacelle.py - tractor motor nacelle on a streamlined pylon.
# The pylon is a visible airfoil-section strut connecting the wing
# underside to the nacelle pod. The nacelle body extends mostly forward
# of the wing, with the prop well ahead of the leading edge.
#
# Local frame: nacelle origin = pylon top (wing underside).
# +X = forward (toward prop). Pylon at local x=0.
# =====================================================================
import adsk.core, adsk.fusion, math
from . import common, prop


def build(parent_comp, side, P, apps):
    name = 'Nacelle_R' if side > 0 else 'Nacelle_L'

    # Spanwise station and wing-relative attachment point
    half_span     = P['wingspan_cm'] / 2.0
    sweep_x_per_y = -math.tan(math.radians(P['sweep_deg']))
    dihed_z_per_y =  math.tan(math.radians(P['dihedral_deg']))

    y_pos = side * P['nacelle_y_cm']
    y_abs = abs(y_pos)
    t_norm = y_abs / half_span
    local_chord = (P['root_chord_cm'] * (1 - t_norm)
                   + P['tip_chord_cm'] * t_norm)

    # Wing reference at this station
    qc_x_local = sweep_x_per_y * y_abs
    z_local    = dihed_z_per_y * y_abs
    le_x       = qc_x_local + 0.75 * local_chord    # wing leading edge

    # Nacelle attachment: pylon hangs straight down from the wing
    # underside ~25% chord aft of the LE (plenty of wing thickness there)
    pylon_aft_of_le = local_chord * 0.25
    nacelle_origin_x = le_x - pylon_aft_of_le

    occ = common.new_component(parent_comp, name,
                               x=nacelle_origin_x, y=y_pos, z=z_local)
    comp = occ.component

    # ---------------- PYLON (airfoil-section strut) ----------------
    pylon_chord = P['nacelle_pylon_chord_cm']
    pylon_thick = P['nacelle_pylon_thick_cm']
    pylon_drop  = P['nacelle_drop_cm']

    # Sketch the pylon airfoil cross-section on the XY plane (Z=0)
    pySk = comp.sketches.add(comp.xYConstructionPlane)
    npts_p = 16 if pylon_chord < 20 else 40
    raw = common.naca4(0, 0, 12, pylon_chord, n=npts_p, te_thick_pct=1.5)
    qc = pylon_chord * 0.25
    # Scale thickness to match pylon_thick
    pylon_pts = [(qc - x, z * (pylon_thick / (pylon_chord * 0.12)))
                 for (x, z) in raw]
    common.add_closed_spline(pySk, pylon_pts)

    # Extrude downward in -Z direction
    prof = pySk.profiles.item(0)
    feats = comp.features.extrudeFeatures
    ei = feats.createInput(prof,
                           adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    extent = adsk.fusion.DistanceExtentDefinition.create(
        adsk.core.ValueInput.createByReal(pylon_drop))
    ei.setOneSideExtent(extent,
                        adsk.fusion.ExtentDirections.NegativeExtentDirection)
    try:
        pylon_feat = feats.add(ei)
        pylon_body = pylon_feat.bodies.item(0)
    except:
        # Fallback: use BRep box as pylon strut
        pylon_body = common.make_box(
            comp,
            (0, 0, -pylon_drop / 2),
            (pylon_chord, pylon_thick, pylon_drop),
            name='Pylon_Fallback')
    pylon_body.name = 'Pylon'
    common.apply_app(pylon_body, apps['nacelle'])

    # ---------------- NACELLE BODY (mostly forward of pylon) ----------
    nac_len = P['nacelle_len_cm']
    nac_r   = P['nacelle_r_cm']
    body_z  = -pylon_drop - nac_r * 0.4

    body_x_aft = -nac_len * 0.20
    body_x_fwd =  nac_len * 0.80

    # Revolved half-profile
    nbSk = comp.sketches.add(comp.xZConstructionPlane)
    pts = [
        (body_x_aft,                              0.0),
        (body_x_aft + nac_len * 0.10,    nac_r * 0.55),
        (body_x_aft + nac_len * 0.30,    nac_r * 0.92),
        (body_x_aft + nac_len * 0.55,    nac_r),
        (body_x_aft + nac_len * 0.85,    nac_r * 0.85),
        (body_x_fwd,                              0.0),
    ]
    spts = adsk.core.ObjectCollection.create()
    for (x, z) in pts:
        spts.add(adsk.core.Point3D.create(x, z + body_z, 0))
    nbSk.sketchCurves.sketchFittedSplines.add(spts)
    nbSk.sketchCurves.sketchLines.addByTwoPoints(
        adsk.core.Point3D.create(body_x_aft, body_z, 0),
        adsk.core.Point3D.create(body_x_fwd, body_z, 0))

    # Construction axis for revolve at the nacelle centerline
    axisSk = comp.sketches.add(comp.xZConstructionPlane)
    axis_line = axisSk.sketchCurves.sketchLines.addByTwoPoints(
        adsk.core.Point3D.create(body_x_aft - 5, body_z, 0),
        adsk.core.Point3D.create(body_x_fwd + 5, body_z, 0))
    common.revolve_full(comp, nbSk.profiles.item(0), axis_line, 'new')
    nac_body = comp.bRepBodies.item(comp.bRepBodies.count - 1)
    nac_body.name = name + '_Body'
    common.apply_app(nac_body, apps['nacelle'])

    # ---------------- Cooling intake (cosmetic detail) -----------------
    try:
        common.make_box(comp,
                        (body_x_aft + nac_len * 0.18, 0.0,
                         body_z + nac_r * 0.95),
                        (nac_len * 0.18, nac_r * 0.6, nac_r * 0.12),
                        name='Intake')
        common.apply_app(comp.bRepBodies.item(comp.bRepBodies.count - 1),
                         apps['metal'])
    except:
        pass

    # ---------------- Propeller at nacelle nose ---------------
    prop_x = body_x_fwd + 1.0
    prop.build(comp, P, apps,
               name=('Prop_R' if side > 0 else 'Prop_L'),
               x=prop_x, y=0.0, z=body_z)

    return occ
