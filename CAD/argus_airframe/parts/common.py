# =====================================================================
# parts/common.py — shared geometry helpers and the appearance palette
# All units are CENTIMETERS (Fusion API internal unit).
# =====================================================================
import adsk.core, adsk.fusion, math, traceback


# ============================ AIRFOIL ================================
def naca4(m_pct, p_tenths, t_pct, chord_cm, n=40, flip_z=False,
          te_thick_pct=0.8):
    """NACA 4-digit airfoil as a closed loop (TE -> upper -> LE -> lower -> TE).
    Returns list of (x, z) tuples in cm.
    te_thick_pct: leaves a small finite trailing-edge thickness so lofts can
      form a clean TE surface (real airfoils have ~0.5-1.5%). Default 0.8%."""
    m = m_pct / 100.0
    p = p_tenths / 10.0
    t = t_pct / 100.0
    upper, lower = [], []
    for i in range(n + 1):
        beta = math.pi * i / n
        x = (1 - math.cos(beta)) / 2.0
        yt = 5*t*(0.2969*math.sqrt(x) - 0.1260*x - 0.3516*x*x
                  + 0.2843*x*x*x - 0.1036*x*x*x*x)
        if 0 < x < p and p > 0:
            yc  = (m/p**2) * (2*p*x - x*x)
            dyc = (2*m/p**2) * (p - x)
        elif p > 0:
            yc  = (m/(1-p)**2) * ((1 - 2*p) + 2*p*x - x*x)
            dyc = (2*m/(1-p)**2) * (p - x)
        else:
            yc, dyc = 0, 0
        th = math.atan(dyc)
        upper.append((x - yt*math.sin(th), yc + yt*math.cos(th)))
        lower.append((x + yt*math.sin(th), yc - yt*math.cos(th)))

    # Open the trailing edge — quadratic blend so LE region is unaffected
    if te_thick_pct > 0:
        te_h = te_thick_pct / 100.0 / 2.0    # half-thickness, normalized
        upper = [(x, z + te_h * x * x) for (x, z) in upper]
        lower = [(x, z - te_h * x * x) for (x, z) in lower]

    sign = -1 if flip_z else 1
    # Build a properly closed loop:
    #   TE upper → LE (via upper surface) → TE lower (via lower surface)
    # The spline's isClosed=True will then connect TE lower → TE upper,
    # forming the trailing edge. Including lower[-1] (TE lower point) is
    # critical — without it the TE has no closing segment and the profile
    # is open, which means the loft has nothing to work with.
    loop = list(reversed(upper)) + lower[1:]   # include TE lower point
    pts = [(x*chord_cm, sign*z*chord_cm) for (x, z) in loop]
    return pts


def add_closed_spline(sketch, pts2d):
    """Create a closed airfoil profile from a loop of (u, v) tuples.
    Uses an OPEN fitted spline + a straight closing line at the trailing
    edge.  This is far more reliable than isClosed=True, which often
    fails to produce a valid Fusion profile for lofting."""
    pts = adsk.core.ObjectCollection.create()
    for (u, v) in pts2d:
        pts.add(adsk.core.Point3D.create(u, v, 0))
    s = sketch.sketchCurves.sketchFittedSplines.add(pts)
    # Close the profile with a straight line from last point to first
    # (trailing-edge closure). This mirrors how the fuselage revolve
    # profile is built (spline + axis line) which Fusion handles reliably.
    p_first = adsk.core.Point3D.create(pts2d[0][0], pts2d[0][1], 0)
    p_last  = adsk.core.Point3D.create(pts2d[-1][0], pts2d[-1][1], 0)
    sketch.sketchCurves.sketchLines.addByTwoPoints(p_last, p_first)
    return s


def add_polygon(sketch, pts2d, close=True):
    """Add a polyline as connected sketch lines."""
    lines = sketch.sketchCurves.sketchLines
    pts = [adsk.core.Point3D.create(u, v, 0) for (u, v) in pts2d]
    n = len(pts)
    last = n if close else n - 1
    created = []
    for i in range(last):
        created.append(lines.addByTwoPoints(pts[i], pts[(i+1) % n]))
    return created


# ====================== COMPONENT / TRANSFORMS ========================
def new_component(parent_comp, name, x=0.0, y=0.0, z=0.0,
                  rx_deg=0.0, ry_deg=0.0, rz_deg=0.0):
    """Create a new sub-component under parent_comp at the given world pose.
    Returns the Occurrence."""
    t = adsk.core.Matrix3D.create()

    # Rotations applied in Z, Y, X order (yaw, pitch, roll)
    if rz_deg:
        rz = adsk.core.Matrix3D.create()
        rz.setToRotation(math.radians(rz_deg),
                         adsk.core.Vector3D.create(0, 0, 1),
                         adsk.core.Point3D.create(0, 0, 0))
        t.transformBy(rz)
    if ry_deg:
        ry = adsk.core.Matrix3D.create()
        ry.setToRotation(math.radians(ry_deg),
                         adsk.core.Vector3D.create(0, 1, 0),
                         adsk.core.Point3D.create(0, 0, 0))
        t.transformBy(ry)
    if rx_deg:
        rx = adsk.core.Matrix3D.create()
        rx.setToRotation(math.radians(rx_deg),
                         adsk.core.Vector3D.create(1, 0, 0),
                         adsk.core.Point3D.create(0, 0, 0))
        t.transformBy(rx)

    t.translation = adsk.core.Vector3D.create(x, y, z)

    occ = parent_comp.occurrences.addNewComponent(t)
    occ.component.name = name
    return occ


def offset_plane(comp, base_plane, offset_cm):
    pin = comp.constructionPlanes.createInput()
    pin.setByOffset(base_plane,
                    adsk.core.ValueInput.createByReal(offset_cm))
    return comp.constructionPlanes.add(pin)


# ============================= FEATURES ==============================
def loft_solid(comp, profiles, op='new'):
    """Loft through a list of profiles. op = 'new' | 'join'."""
    feats = comp.features.loftFeatures
    fop = (adsk.fusion.FeatureOperations.NewBodyFeatureOperation
           if op == 'new' else
           adsk.fusion.FeatureOperations.JoinFeatureOperation)
    li = feats.createInput(fop)
    for p in profiles:
        li.loftSections.add(p)
    li.isSolid = True
    return feats.add(li)


def revolve_full(comp, profile, axis, op='new'):
    """Revolve a profile 360° around an axis."""
    feats = comp.features.revolveFeatures
    fop = (adsk.fusion.FeatureOperations.NewBodyFeatureOperation
           if op == 'new' else
           adsk.fusion.FeatureOperations.JoinFeatureOperation)
    ri = feats.createInput(profile, axis, fop)
    ri.setAngleExtent(False, adsk.core.ValueInput.createByReal(2*math.pi))
    return feats.add(ri)


def extrude_distance(comp, profile, distance_cm,
                     symmetric=False, op='new'):
    feats = comp.features.extrudeFeatures
    fop = (adsk.fusion.FeatureOperations.NewBodyFeatureOperation
           if op == 'new' else
           adsk.fusion.FeatureOperations.JoinFeatureOperation if op == 'join' else
           adsk.fusion.FeatureOperations.CutFeatureOperation)
    ei = feats.createInput(profile, fop)
    if symmetric:
        ei.setSymmetricExtent(adsk.core.ValueInput.createByReal(distance_cm/2),
                              True)
    else:
        ei.setDistanceExtent(False,
                             adsk.core.ValueInput.createByReal(distance_cm))
    return feats.add(ei)


def make_sphere(comp, center_xyz, radius_cm, name=None):
    """Create a true BRep sphere via the temporary BRep manager — works
    regardless of axis-crossing rules that constrain revolves."""
    tbm = adsk.fusion.TemporaryBRepManager.get()
    cx, cy, cz = center_xyz
    body = tbm.createSphere(adsk.core.Point3D.create(cx, cy, cz), radius_cm)
    added = comp.bRepBodies.add(body)
    if name:
        added.name = name
    return added


def make_cylinder(comp, p1_xyz, p2_xyz, radius_cm, name=None):
    """Create a true BRep cylinder between two points."""
    tbm = adsk.fusion.TemporaryBRepManager.get()
    p1 = adsk.core.Point3D.create(*p1_xyz)
    p2 = adsk.core.Point3D.create(*p2_xyz)
    body = tbm.createCylinderOrCone(p1, radius_cm, p2, radius_cm)
    added = comp.bRepBodies.add(body)
    if name:
        added.name = name
    return added


def make_cone(comp, base_xyz, base_r_cm, tip_xyz, tip_r_cm, name=None):
    tbm = adsk.fusion.TemporaryBRepManager.get()
    body = tbm.createCylinderOrCone(
        adsk.core.Point3D.create(*base_xyz), base_r_cm,
        adsk.core.Point3D.create(*tip_xyz), tip_r_cm)
    added = comp.bRepBodies.add(body)
    if name:
        added.name = name
    return added


def _norm3(v):
    m = math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
    return v if m == 0 else (v[0]/m, v[1]/m, v[2]/m)


def make_oriented_box(comp, center_xyz, length_dir, width_dir,
                      sx, sy, sz, name=None):
    """Box centered at center_xyz with custom orientation.
    Direction vectors are orthonormalized internally (Fusion requires them
    to be exactly perpendicular, so callers can pass approximate ones)."""
    L = _norm3(length_dir)
    dot = L[0]*width_dir[0] + L[1]*width_dir[1] + L[2]*width_dir[2]
    w_orth = (width_dir[0] - dot*L[0],
              width_dir[1] - dot*L[1],
              width_dir[2] - dot*L[2])
    W = _norm3(w_orth)
    tbm = adsk.fusion.TemporaryBRepManager.get()
    cx, cy, cz = center_xyz
    obb = adsk.core.OrientedBoundingBox3D.create(
        adsk.core.Point3D.create(cx, cy, cz),
        adsk.core.Vector3D.create(*L),
        adsk.core.Vector3D.create(*W),
        sx, sy, sz)
    body = tbm.createBox(obb)
    added = comp.bRepBodies.add(body)
    if name:
        added.name = name
    return added


def make_box(comp, center_xyz, size_xyz, name=None):
    """Axis-aligned box centered at center_xyz with given (sx, sy, sz)."""
    tbm = adsk.fusion.TemporaryBRepManager.get()
    cx, cy, cz = center_xyz
    sx, sy, sz = size_xyz
    obb = adsk.core.OrientedBoundingBox3D.create(
        adsk.core.Point3D.create(cx, cy, cz),
        adsk.core.Vector3D.create(1, 0, 0),
        adsk.core.Vector3D.create(0, 1, 0),
        sx, sy, sz)
    body = tbm.createBox(obb)
    added = comp.bRepBodies.add(body)
    if name:
        added.name = name
    return added


def fillet_all_edges(comp, body, radius_cm):
    """Try to fillet every edge of a body; ignore failures."""
    try:
        edges = adsk.core.ObjectCollection.create()
        for e in body.edges:
            edges.add(e)
        feats = comp.features.filletFeatures
        fi = feats.createInput()
        fi.addConstantRadiusEdgeSet(
            edges, adsk.core.ValueInput.createByReal(radius_cm), True)
        feats.add(fi)
    except:
        pass


# ============================ APPEARANCES ============================
def _try_get_lib_appearance(app, lib_names, app_names):
    for lname in lib_names:
        try:
            lib = app.materialLibraries.itemByName(lname)
        except:
            lib = None
        if not lib:
            continue
        for an in app_names:
            try:
                a = lib.appearances.itemByName(an)
            except:
                a = None
            if a:
                return a
    return None


def get_appearance(app, design, name_candidates):
    """Look up an appearance by trying several library names in order.
    Returns an Appearance ready to assign, or None on failure."""
    lib_names = ['Fusion 360 Appearance Library',
                 'Fusion Appearance Library']
    src = _try_get_lib_appearance(app, lib_names, name_candidates)
    if not src:
        return None
    for nm in name_candidates:
        try:
            existing = design.appearances.itemByName(nm)
        except:
            existing = None
        if existing:
            return existing
    try:
        return design.appearances.addByCopy(src, src.name + '_argus')
    except:
        return src


def apply_app(target, appearance):
    if appearance is None:
        return
    try:
        target.appearance = appearance
    except:
        pass


def build_appearance_palette(app, design):
    """Single source of truth for all material appearances used in ARGUS."""
    return {
        # Composite skin — matte white
        'skin':       get_appearance(app, design, [
            'Plastic - Matte (White)', 'Plastic - Glossy (White)',
            'Paint - Enamel Glossy (White)', 'Plastic - Matte']),
        # Solar PV cells — deep dark blue
        'pv_cell':    get_appearance(app, design, [
            'Paint - Enamel Glossy (Dark Blue)',
            'Plastic - Matte (Dark Blue)', 'Plastic - Matte (Blue)',
            'Plastic - Glossy (Blue)']),
        # Bus bars between cells — silver
        'busbar':     get_appearance(app, design, [
            'Aluminum - Polished', 'Aluminum - Satin', 'Steel - Polished']),
        # Carbon-fiber tubes (booms, prop blades)
        'carbon':     get_appearance(app, design, [
            'Carbon Fiber', 'Plastic - Matte (Black)']),
        # Nacelle housings — anodized gray
        'nacelle':    get_appearance(app, design, [
            'Aluminum - Anodized Glossy (Gray)',
            'Plastic - Matte (Gray)', 'Plastic - Matte']),
        # Spinner / hub
        'spinner':    get_appearance(app, design, [
            'Plastic - Matte (Black)', 'Paint - Enamel Glossy (Black)']),
        # Sensor pod body — black ISR housing
        'pod':        get_appearance(app, design, [
            'Plastic - Matte (Black)', 'Paint - Enamel Glossy (Black)']),
        # EO/SWIR optical glass with AR coating
        'optic':      get_appearance(app, design, [
            'Glass - Clear', 'Plastic - Translucent Matte (Gray)']),
        # Anodized metal — gimbal mounts, lens barrels
        'metal':      get_appearance(app, design, [
            'Aluminum - Anodized Glossy (Black)',
            'Aluminum - Polished', 'Steel - Satin']),
        # Microphone foam windscreens
        'foam':       get_appearance(app, design, [
            'Plastic - Matte (Gray)', 'Plastic - Matte']),
        # SatCom radome — beige composite
        'radome':     get_appearance(app, design, [
            'Plastic - Matte (Beige)', 'Plastic - Matte (White)',
            'Plastic - Matte']),
    }
