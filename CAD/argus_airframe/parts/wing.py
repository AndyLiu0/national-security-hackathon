# =====================================================================
# parts/wing.py — single wing half (lofted NACA 4412 with sweep, dihedral,
#                 washout). Camber ON TOP. Use side=+1 (right) or -1 (left).
# =====================================================================
import adsk.core, adsk.fusion, math
from . import common


def build(parent_comp, side, P, apps, build_solar=True):
    """Build one wing half as a sub-component. Returns the Occurrence."""
    name = 'Wing_R' if side > 0 else 'Wing_L'
    occ = common.new_component(parent_comp, name)
    comp = occ.component

    half_span = P['wingspan_cm'] / 2.0
    sweep_x   = -half_span * math.tan(math.radians(P['sweep_deg']))
    # Note: sketch-Y on xZ plane maps to world -Z, so we negate dihedral
    # here to get a real positive-dihedral wing (tips UP in world).
    dihed_z   = -half_span * math.tan(math.radians(P['dihedral_deg']))
    tip_y     =  side * half_span

    # ------------- Root profile (XZ plane, Y=0) -------------
    rootSk = comp.sketches.add(comp.xZConstructionPlane)
    # Use fewer spline points for small chords to avoid degenerate geometry
    npts = 24 if P['root_chord_cm'] < 50 else 40
    raw_root = common.naca4(P['airfoil_camber_pct'],
                            P['airfoil_camber_pos'],
                            P['airfoil_thick_pct'],
                            P['root_chord_cm'],
                            n=npts,
                            flip_z=True)   # camber on TOP (+Z up)
    # Map airfoil coords (LE @ x=0, TE @ x=chord) into sketch coords:
    #   sketch_u = chord/4 - x   →  LE forward (+u), TE aft (-u),
    #                                quarter-chord at sketch origin
    #   sketch_v = z              →  upper surface points to +Z (UP)
    qc_root = P['root_chord_cm'] * 0.25
    root_pts = [(qc_root - x, z) for (x, z) in raw_root]
    common.add_closed_spline(rootSk, root_pts)

    # ------------- Tip plane offset along Y -------------
    tipPlane = common.offset_plane(comp, comp.xZConstructionPlane, tip_y)
    tipSk = comp.sketches.add(tipPlane)

    npts_tip = 24 if P['tip_chord_cm'] < 50 else 40
    raw_tip = common.naca4(P['airfoil_camber_pct'],
                           P['airfoil_camber_pos'],
                           P['airfoil_thick_pct'],
                           P['tip_chord_cm'],
                           n=npts_tip,
                           flip_z=True)
    qc_tip = P['tip_chord_cm'] * 0.25
    # Airfoil-local centered at quarter-chord
    tip_local = [(qc_tip - x, z) for (x, z) in raw_tip]
    # Apply washout — LE drops in world (positive sketch v = world -Z = down)
    wash = math.radians(P['washout_deg'])
    cw, sw = math.cos(wash), math.sin(wash)
    tip_pts = []
    for (u, v) in tip_local:
        ur = u*cw - v*sw
        vr = u*sw + v*cw
        # Then translate by sweep and dihedral
        tip_pts.append((ur + sweep_x, vr + dihed_z))
    common.add_closed_spline(tipSk, tip_pts)

    # ------------- Loft -------------
    loft = common.loft_solid(
        comp, [rootSk.profiles.item(0), tipSk.profiles.item(0)], 'new')
    skin = loft.bodies.item(0)
    skin.name = name + '_Skin'
    common.apply_app(skin, apps['skin'])

    # ------------- Solar array (on upper face) -------------
    if build_solar:
        try:
            from . import solar_array
            solar_array.build(occ, side, P, apps, skin)
        except Exception:
            # If solar fails, paint the upper faces as a fallback so the
            # wing still looks like a PV surface.
            _paint_upper_faces(skin, apps.get('pv_cell'))

    return occ


def _paint_upper_faces(body, pv_app):
    if pv_app is None:
        return
    for face in body.faces:
        try:
            ok, n = face.evaluator.getNormalAtPoint(face.pointOnFace)
            if ok and n.z > 0.35:
                face.appearance = pv_app
        except:
            pass
