# =====================================================================
# parts/sensor_pod.py — slung sensor pod beneath the fuselage nose.
# This is a sub-assembly: it owns the pod body + pylon, and hosts
# child sub-components (EO gimbal, SWIR camera, mics, pressure ports).
#
# Pod local frame origin = pod centroid. +X = forward, +Z = up.
# =====================================================================
import adsk.core, adsk.fusion, math
from . import common, payload, fittings


def build(parent_comp, P, apps):
    occ = common.new_component(parent_comp, 'Sensor_Pod',
                               x=P['pod_forward_cm'],
                               y=0,
                               z=-P['pod_drop_cm'])
    comp = occ.component

    L = P['pod_length_cm']
    R = P['pod_radius_cm']

    # ---------------- Pod body (revolved capsule) ------------------
    sk = comp.sketches.add(comp.xZConstructionPlane)
    pts = [
        (-L/2,         0.0),
        (-L/2 + R*0.5, R*0.85),
        (-L/4,         R),
        ( L/4,         R),
        ( L/2 - R*0.5, R*0.85),
        ( L/2,         0.0),
    ]
    spts = adsk.core.ObjectCollection.create()
    for (x, z) in pts:
        spts.add(adsk.core.Point3D.create(x, z, 0))
    sk.sketchCurves.sketchFittedSplines.add(spts)
    sk.sketchCurves.sketchLines.addByTwoPoints(
        adsk.core.Point3D.create(-L/2, 0, 0),
        adsk.core.Point3D.create( L/2, 0, 0))
    common.revolve_full(comp, sk.profiles.item(0),
                        comp.xConstructionAxis, 'new')
    pod_body = comp.bRepBodies.item(comp.bRepBodies.count - 1)
    pod_body.name = 'Pod_Body'
    common.apply_app(pod_body, apps['pod'])

    # ---------------- Pylon (airfoil-section strut up to fuselage) -
    pylon_chord = P['pod_pylon_chord_cm']
    pylon_thick = P['pod_pylon_thick_cm']
    pylon_top_z = R * 0.5  # starts inside the pod top, extends up

    pyPlane = common.offset_plane(comp, comp.xYConstructionPlane,
                                  pylon_top_z)
    pySk = comp.sketches.add(pyPlane)
    raw = common.naca4(0, 0, 12, pylon_chord)
    qc = pylon_chord * 0.25
    pylon_pts = [(qc - x, z * (pylon_thick / (pylon_chord * 0.12)))
                 for (x, z) in raw]
    common.add_closed_spline(pySk, pylon_pts)
    common.extrude_distance(comp, pySk.profiles.item(0),
                            P['pod_drop_cm'] - pylon_top_z, op='new')
    pylon_body = comp.bRepBodies.item(comp.bRepBodies.count - 1)
    pylon_body.name = 'Pod_Pylon'
    common.apply_app(pylon_body, apps['skin'])

    # ---------------- EO gimbal (chin turret, flush with pod belly) -
    # Mount plate sits at the pod's lower surface (z = -R) so the yoke
    # arms protrude DOWN from the pod and clearly attach to it.
    payload.build_eo_gimbal(
        comp, P, apps,
        pos_xyz=(L * 0.18, 0, -R),
        name='EO_Gimbal')

    # ---------------- SWIR camera (forward, looking out pod nose) --
    payload.build_swir_camera(
        comp, P, apps,
        pos_xyz=(L * 0.32, 0, R * 0.0),
        name='SWIR_Camera')

    # ---------------- Microphone array on pod sides (TDOA local) --
    # Two mics on each side, fore/aft, pointing outward (+/-Y).
    mic_x_fwd = L * 0.25
    mic_x_aft = -L * 0.20
    for s, axis_lbl in ((+1, '+y'), (-1, '-y')):
        for (x, lbl) in ((mic_x_fwd, 'Fwd'), (mic_x_aft, 'Aft')):
            fittings.build_mic(
                comp, P, apps,
                pos_xyz=(x, s * R * 0.95, 0),
                axis=axis_lbl,
                name=f'Mic_{"R" if s>0 else "L"}_{lbl}')

    # ---------------- Pressure ports on pod nose (Pitot pair) -----
    # Two probes flanking the pod nose, pointing forward (+X)
    for s, lbl in ((+1, 'R'), (-1, 'L')):
        fittings.build_pressure_port(
            comp, P, apps,
            pos_xyz=(L * 0.46, s * R * 0.5, 0),
            axis='+x',
            name=f'Pitot_{lbl}')

    return occ
