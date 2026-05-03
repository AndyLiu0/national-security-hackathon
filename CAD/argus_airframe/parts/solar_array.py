# =====================================================================
# parts/solar_array.py — visible PV cell grid bonded to the wing's
# upper surface. Cells are oriented to follow the wing's local
# chord direction AND its dihedral tilt (and small washout twist),
# so the grid hugs the surface instead of floating as a flat XY plane.
# =====================================================================
import adsk.core, adsk.fusion, math
from . import common


def build(parent_wing_occ, side, P, apps, wing_skin_body):
    parent_comp = parent_wing_occ.component
    occ = common.new_component(parent_comp,
                               'Solar_Array_R' if side > 0 else 'Solar_Array_L')
    comp = occ.component

    # Paint wing skin's upper faces with PV colour so gaps still read PV
    pv = apps.get('pv_cell')
    if pv:
        for face in wing_skin_body.faces:
            try:
                ok, n = face.evaluator.getNormalAtPoint(face.pointOnFace)
                if ok and n.z > 0.30:
                    face.appearance = pv
            except:
                pass

    # ---- Layout params ----
    half_span    = P['wingspan_cm'] / 2.0
    root_chord   = P['root_chord_cm']
    tip_chord    = P['tip_chord_cm']
    cell         = P['solar_cell_cm']
    gap          = P['solar_cell_gap_cm']
    inset_x      = P['solar_inset_cm']
    inset_y_root = P['solar_inset_root_cm']
    inset_y_tip  = P['solar_inset_tip_cm']
    tile_thick   = P['solar_tile_thickness_cm']
    thick_pct    = P['airfoil_thick_pct'] / 100.0

    sweep_x_per_y = -math.tan(math.radians(P['sweep_deg']))
    dihed_z_per_y =  math.tan(math.radians(P['dihedral_deg']))

    # ---- Per-row geometry ----
    y0 = inset_y_root
    y1 = half_span - inset_y_tip
    n_rows = int((y1 - y0) // (cell + gap))

    for row in range(n_rows):
        # Spanwise station of this row's center, signed by side
        y_signed = side * (y0 + (row + 0.5) * (cell + gap))
        y_abs    = abs(y_signed)
        t_norm   = y_abs / half_span

        local_chord = root_chord * (1 - t_norm) + tip_chord * t_norm

        # Wing reference frame at this station:
        # - Quarter-chord X (sweep applied)
        # - Surface lift Z (dihedral lifts tip UP in world; positive Z)
        qc_x_local  = sweep_x_per_y * y_abs
        z_chordline = dihed_z_per_y * y_abs
        # Lift cells onto the upper surface — cleared by camber + thickness
        # (~ camber% + half-thickness%) above the chord line at mid-chord
        z_surface   = z_chordline + local_chord * (
            P['airfoil_camber_pct']/100.0 + thick_pct * 0.55)

        # Local twist (washout) at this station — small, but include it
        wash_local = -P['washout_deg'] * t_norm  # negative = LE down
        cw, sw = math.cos(math.radians(wash_local)), math.sin(math.radians(wash_local))

        # Local dihedral angle, signed for which wing
        d_signed = math.radians(P['dihedral_deg']) * (1 if side > 0 else -1)
        cd, sd = math.cos(d_signed), math.sin(d_signed)

        # Direction vectors for the cell tile:
        # - length along chord (X), tilted slightly by washout: (cos w, 0, -sin w)
        # - width along span (Y), tilted by dihedral: (0, sign*cos d, |sin d|)
        #   Note: sin(d_signed) carries the correct sign — both wingtips
        #   lift up toward +Z, so we need sin(|d|) for the Z component.
        length_dir = (cw, 0, -sw)
        width_dir  = (0, math.copysign(cd, side), abs(sd))

        # Cell-grid extent at this station — chordwise from LE inset to TE inset
        x_le_aft  = qc_x_local + 0.25 * local_chord - inset_x
        x_te_fwd  = qc_x_local - 0.75 * local_chord + inset_x
        avail = x_le_aft - x_te_fwd
        n_cols = int((avail + gap) // (cell + gap))
        if n_cols <= 0:
            continue

        # March cells from LE side aft toward TE
        for col in range(n_cols):
            xc_chordline = x_le_aft - cell/2 - col * (cell + gap)
            if xc_chordline - cell/2 < x_te_fwd:
                break
            # Cell center: chord-direction position xc, span-direction y_signed,
            # plus the cell's center-of-thickness above the surface
            cx = xc_chordline
            cy = y_signed
            cz = z_surface + tile_thick/2

            common.make_oriented_box(
                comp,
                (cx, cy, cz),
                length_dir, width_dir,
                cell, cell, tile_thick,
                name=f'PVCell_{row:02d}_{col:02d}')

        # Silver bus-bar strip just outboard of this row
        if row < n_rows - 1:
            busbar_y = side * (y0 + (row + 1) * (cell + gap) - gap/2)
            common.make_oriented_box(
                comp,
                (qc_x_local, busbar_y,
                 z_surface + tile_thick/2),
                length_dir, width_dir,
                avail * 0.95, gap * 0.6, tile_thick * 0.6,
                name=f'BusBar_{row:02d}')

    # Apply appearances
    for body in comp.bRepBodies:
        if body.name.startswith('PVCell'):
            common.apply_app(body, apps.get('pv_cell'))
        elif body.name.startswith('BusBar'):
            common.apply_app(body, apps.get('busbar'))

    return occ
