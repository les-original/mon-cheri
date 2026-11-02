#!/usr/bin/env python3
"""Build Tier A mon-cheri shapes and update FCStd + STLs."""
import os
import math
import traceback
import FreeCAD as App
import Part
import MeshPart

GUARD = "/tmp/tier_a_guard"
if os.path.exists(GUARD):
    raise SystemExit(0)
open(GUARD, "w").close()

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DOC_PATH = f"{REPO}/src/mon-cheri.FCStd"
LOG_PATH = "/tmp/tier_a_log.txt"

def log(*a):
    line = " ".join(str(x) for x in a)
    print(line, flush=True)
    with open(LOG_PATH, "a") as f:
        f.write(line + "\n")

open(LOG_PATH, "w").close()

CABLE_R = 1.0
LIP_R = 0.175
OUTER_R = 1.0
CONE_H = 1.0 / math.tan(math.radians(59))
# After the 1 mm outside fillet, the cable mouth sits this far in from the flat side (matches square-m3).
MOUTH_INSET = 0.175

KEEP = {
    "tri-m3": 287.487,
    "tri-m4": 396.422,
    "ball-m3": 140.597,
    "ball-m4": 208.352,
    "barrel-m3": 242.173,
    "barrel-m4": 240.469,
    "square-m3": 653.160,
    "square-m4": 899.363,
    "heart-m3": None,
    "heart-m4": None,
}


def unit(v):
    v = App.Vector(v)
    if v.Length < 1e-12:
        raise ValueError("zero vector")
    v.normalize()
    return v


def curve_of(edge):
    try:
        return edge.Curve
    except Exception:
        return None


def vertical_fillet(solid, thick, radius=OUTER_R):
    verts = [
        e
        for e in solid.Edges
        if e.BoundBox.ZLength > thick * 0.45 and e.Length > 0.2
    ]
    if verts:
        try:
            solid = solid.makeFillet(radius, verts)
        except Exception:
            pass
    for z in (0.0, -thick):
        ring = [
            e
            for e in solid.Edges
            if abs(e.BoundBox.ZMin - z) < 0.06
            and abs(e.BoundBox.ZMax - z) < 0.06
            and e.Length > 0.35
        ]
        if ring:
            try:
                solid = solid.makeFillet(radius, ring)
            except Exception:
                pass
    return solid


def rot_z(v, deg):
    t = math.radians(deg)
    c, s = math.cos(t), math.sin(t)
    return App.Vector(v.x * c - v.y * s, v.x * s + v.y * c, v.z)


def outer_fillet(solid, thick, radius=OUTER_R):
    return vertical_fillet(solid, thick, radius)


def cable_circles_at_zmid(solid, zmid):
    circles = []
    for e in solid.Edges:
        c = curve_of(e)
        if c is None or type(c).__name__ != "Circle":
            continue
        if abs(c.Radius - CABLE_R) > 0.05 or abs(c.Center.z - zmid) > 0.12:
            continue
        circles.append(e)
    return circles


def lip_fillet_drop(solid, cable_center, bolt_r, zmid):
    """Drop: screw lip; cable mouth is a partial arc so skip cable torus (avoids bad fillets)."""
    screw_e = []
    for e in solid.Edges:
        c = curve_of(e)
        if c is None or type(c).__name__ != "Circle":
            continue
        cc = c.Center
        if abs(c.Radius - bolt_r) < 0.06 and abs(cc.x) < 0.2 and abs(cc.y) < 0.2:
            if abs(cc.z) < 0.15 and abs(e.Length - 2 * math.pi * bolt_r) < 0.15:
                screw_e.append(e)
    if len(screw_e) != 1:
        raise RuntimeError(f"drop screw lip edges {len(screw_e)}")
    solid = solid.makeFillet(LIP_R, screw_e)
    try:
        solid = solid.removeSplitter()
    except Exception:
        pass
    return solid


def lip_fillet_by_center(solid, cable_center, screw_center, bolt_r, zmid, mouth_x_min=0.0):
    """Cable lip on curved or barrel sides (bean); picks mouth circle by position."""
    cable_e, screw_e = [], []
    mouth_xy = App.Vector(cable_center.x, cable_center.y, 0)
    for e in solid.Edges:
        c = curve_of(e)
        if c is None or type(c).__name__ != "Circle":
            continue
        cc = c.Center
        if abs(c.Radius - CABLE_R) < 0.05 and abs(cc.z - zmid) < 0.15:
            if cc.x >= mouth_x_min and (
                App.Vector(cc.x, cc.y, 0) - mouth_xy
            ).Length < 1.2:
                cable_e.append(e)
        if abs(c.Radius - bolt_r) < 0.06 and abs(cc.x - screw_center.x) < 0.2 and abs(cc.y - screw_center.y) < 0.2:
            if abs(cc.z) < 0.15 and abs(e.Length - 2 * math.pi * bolt_r) < 0.15:
                screw_e.append(e)
    if not cable_e:
        for e in solid.Edges:
            c = curve_of(e)
            if c is None or type(c).__name__ != "Circle":
                continue
            if abs(c.Radius - CABLE_R) > 0.05:
                continue
            cc = c.Center
            if abs(cc.z - zmid) > 0.15:
                continue
            if (App.Vector(cc.x, cc.y, 0) - mouth_xy).Length > 1.5:
                continue
            cable_e.append(e)
    if not cable_e or len(screw_e) != 1:
        raise RuntimeError(f"lip edges cable={len(cable_e)} screw={len(screw_e)}")
    cable_e.sort(
        key=lambda e: (App.Vector(e.Curve.Center.x, e.Curve.Center.y, 0) - mouth_xy).Length
    )
    try:
        solid = solid.makeFillet(LIP_R, [cable_e[0], screw_e[0]])
    except Exception:
        solid = solid.makeFillet(LIP_R, screw_e)
        solid = solid.makeFillet(LIP_R, [cable_e[0]])
    try:
        solid = solid.removeSplitter()
    except Exception:
        pass
    return solid


def lip_fillet(solid, bolt_r, zmid, outward):
    """Blend the outer cable mouth and top screw opening (square-style)."""
    outward = unit(outward)
    cable_full = [
        e
        for e in cable_circles_at_zmid(solid, zmid)
        if abs(e.Length - 2 * math.pi) < 0.12
    ]
    if not cable_full:
        raise RuntimeError("no full cable mouth circle for lip")
    cable_full.sort(
        key=lambda e: -(
            App.Vector(e.Curve.Center.x, e.Curve.Center.y, 0).dot(outward)
        )
    )
    cable_e = cable_full[0]
    screw_e = []
    for e in solid.Edges:
        c = curve_of(e)
        if c is None or type(c).__name__ != "Circle":
            continue
        cc = c.Center
        if abs(c.Radius - bolt_r) < 0.06 and abs(cc.x) < 0.2 and abs(cc.y) < 0.2:
            if abs(cc.z) < 0.15 and abs(e.Length - 2 * math.pi * bolt_r) < 0.15:
                screw_e.append(e)
    if len(screw_e) != 1:
        raise RuntimeError(f"screw lip edges {len(screw_e)}")
    solid = solid.makeFillet(LIP_R, [cable_e, screw_e[0]])
    try:
        solid = solid.removeSplitter()
    except Exception:
        pass
    return solid


def positive_x_mouth_on_solid(solid, zmid, inset=MOUTH_INSET, prefer_flat_center=True):
    """Outermost +X cable mouth on the zmid slice (after outer fillet), inset for lip."""
    plane = Part.makePlane(30, 30, App.Vector(-15, -15, zmid), App.Vector(0, 0, 1))
    sec = solid.section(plane)
    verts = [v for v in sec.Vertexes if v.X > 0]
    if not verts:
        raise RuntimeError("no +X profile on solid slice")
    if prefer_flat_center:
        max_x = max(v.X for v in verts)
        band = [v for v in verts if v.X >= max_x - 0.35]
        band.sort(key=lambda v: (abs(v.Y), -v.X))
        v = band[0]
    else:
        v = max(verts, key=lambda v: v.X)
    return App.Vector(v.X - inset, v.Y, zmid)


def side_top_pinch(
    solid,
    thick,
    bolt_r,
    face_x=6.0,
    face_y=0.0,
    tilt_deg=0.0,
    verify=True,
    mouth_override=None,
    already_filleted=False,
):
    """Square-style: cable into +X side face, screw down through top center."""
    if not already_filleted:
        solid = outer_fillet(solid, thick)
    zmid = -thick / 2.0
    screw_depth = thick - 1.0
    if mouth_override is not None:
        mouth = App.Vector(mouth_override.x, mouth_override.y, zmid)
        inward = App.Vector(-1, 0, 0)
        if abs(tilt_deg) > 0.01:
            inward = rot_z(inward, tilt_deg)
        inward = unit(inward)
        outward = inward * -1
    else:
        mouth_xy = App.Vector(face_x - MOUTH_INSET, face_y, 0)
        inward = App.Vector(-1, 0, 0)
        if abs(tilt_deg) > 0.01:
            mouth_xy = rot_z(mouth_xy, tilt_deg)
            inward = rot_z(inward, tilt_deg)
        inward = unit(inward)
        outward = inward * -1
        mouth = App.Vector(mouth_xy.x, mouth_xy.y, zmid)
    dist = (mouth - App.Vector(0, 0, zmid)).dot(outward)
    if dist < 2.5:
        raise RuntimeError(f"cable side too shallow {dist:.2f}")
    cable_depth = dist + bolt_r + 0.45
    if cable_depth + CONE_H > 2 * dist - 1.0:
        raise RuntimeError("cable would break out the far side")
    start = mouth - inward * 0.25
    solid = solid.cut(Part.makeCylinder(CABLE_R, cable_depth + 0.25, start, inward))
    cone_at = mouth + inward * cable_depth
    solid = solid.cut(Part.makeCone(CABLE_R, 0.0, CONE_H, cone_at, inward))
    solid = solid.cut(
        Part.makeCylinder(
            bolt_r,
            screw_depth + 0.25,
            App.Vector(0, 0, 0.25),
            App.Vector(0, 0, -1),
        )
    )
    if verify:
        solid = lip_fillet(solid, bolt_r, zmid, outward)
        verify_cable_mouth(solid, zmid, outward, mouth)
    elif mouth_override is not None and already_filleted:
        solid = lip_fillet_drop(solid, mouth, bolt_r, zmid)
    else:
        solid = lip_fillet_by_center(
            solid,
            mouth,
            App.Vector(0, 0, 0),
            bolt_r,
            zmid,
            mouth_x_min=mouth.x - 1.5,
        )
    return solid, mouth, screw_depth


def barrel_pinch(solid, thick, bolt_r, surface_x, surface_y=0.0):
    """Bean / curved +X entry — no flat-face mouth verify."""
    solid = outer_fillet(solid, thick)
    zmid = -thick / 2.0
    screw_depth = thick - 1.0
    cable_center = App.Vector(surface_x, surface_y, zmid)
    dist = abs(surface_x)
    cable_depth = dist + bolt_r + 0.45
    if cable_depth + CONE_H > 2 * dist - 1.0:
        raise RuntimeError("cable breaks out (barrel pinch)")
    start = cable_center - App.Vector(1, 0, 0) * 0.25
    solid = solid.cut(Part.makeCylinder(CABLE_R, cable_depth + 0.25, start, App.Vector(-1, 0, 0)))
    cone_at = cable_center + App.Vector(-1, 0, 0) * cable_depth
    solid = solid.cut(Part.makeCone(CABLE_R, 0.0, CONE_H, cone_at, App.Vector(-1, 0, 0)))
    solid = solid.cut(
        Part.makeCylinder(
            bolt_r,
            screw_depth + 0.25,
            App.Vector(0, 0, 0.25),
            App.Vector(0, 0, -1),
        )
    )
    solid = lip_fillet_by_center(
        solid,
        cable_center,
        App.Vector(0, 0, 0),
        bolt_r,
        zmid,
        mouth_x_min=surface_x - 1.5,
    )
    return solid, cable_center, screw_depth


def verify_cable_mouth(solid, zmid, outward, mouth):
    outward = unit(outward)
    full = [
        e
        for e in cable_circles_at_zmid(solid, zmid)
        if abs(e.Length - 2 * math.pi) < 0.12
    ]
    if not full:
        raise RuntimeError("no full cable mouth circle")
    best = max(
        full,
        key=lambda e: App.Vector(e.Curve.Center.x, e.Curve.Center.y, 0).dot(outward),
    )
    c = best.Curve.Center
    if (App.Vector(c.x, c.y, 0) - App.Vector(mouth.x, mouth.y, 0)).Length > 0.3:
        raise RuntimeError(f"cable mouth at ({c.x:.2f},{c.y:.2f}) expected near ({mouth.x:.2f},{mouth.y:.2f})")
    for e in cable_circles_at_zmid(solid, zmid):
        if abs(e.Length - 2 * math.pi) < 0.12:
            continue
        if e.Length < 0.35:
            continue
        cc = e.Curve.Center
        if App.Vector(cc.x, cc.y, 0).dot(outward) > App.Vector(c.x, c.y, 0).dot(outward) - 0.5:
            raise RuntimeError(f"split cable opening arc length {e.Length:.2f}")


def mesh_topology_check(mesh, name):
    from collections import defaultdict

    topo = mesh.Topology
    pts = topo[0]
    adj = defaultdict(int)
    for fac in topo[1]:
        for a, b in ((fac[0], fac[1]), (fac[1], fac[2]), (fac[2], fac[0])):
            adj[tuple(sorted((a, b)))] += 1
    boundary = sum(1 for n in adj.values() if n != 2)
    if boundary:
        raise RuntimeError(f"{name} mesh has {boundary} open edges")
    if mesh.hasNonManifolds():
        raise RuntimeError(f"{name} non-manifold mesh")
    if mesh.hasSelfIntersections():
        raise RuntimeError(f"{name} self-intersecting mesh")


def hex_wire(flat_to_flat=12.0):
    """Regular hex with flat-to-flat = 12 mm; a flat face on +X for the cable (square-style)."""
    a = flat_to_flat / 2.0  # apothem from origin to the +X flat
    b = a / math.sqrt(3)  # half-length of each flat edge
    # Build with horizontal flats, then rotate so the right flat sits on +X.
    raw = [
        (-b, a),
        (b, a),
        (2 * b, 0),
        (b, -a),
        (-b, -a),
        (-2 * b, 0),
    ]

    def rot90(x, y):
        return App.Vector(y, -x, 0)

    pts = [rot90(x, y) for x, y in raw]
    segs = [Part.LineSegment(pts[i], pts[(i + 1) % 6]).toShape() for i in range(6)]
    return Part.Wire(segs)


def squircle_wire(size=12.0, corner_r=3.0, tilt_deg=8.0):
    h = size / 2.0
    r = min(corner_r, h - 0.5)
    x0, y0 = -h, -h
    x1, y1 = h, h
    edges = [
        Part.LineSegment(App.Vector(x0 + r, y0, 0), App.Vector(x1 - r, y0, 0)).toShape(),
        Part.ArcOfCircle(
            Part.Circle(App.Vector(x1 - r, y0 + r, 0), App.Vector(0, 0, 1), r), -math.pi / 2, 0
        ).toShape(),
        Part.LineSegment(App.Vector(x1, y0 + r, 0), App.Vector(x1, y1 - r, 0)).toShape(),
        Part.ArcOfCircle(
            Part.Circle(App.Vector(x1 - r, y1 - r, 0), App.Vector(0, 0, 1), r), 0, math.pi / 2
        ).toShape(),
        Part.LineSegment(App.Vector(x1 - r, y1, 0), App.Vector(x0 + r, y1, 0)).toShape(),
        Part.ArcOfCircle(
            Part.Circle(App.Vector(x0 + r, y1 - r, 0), App.Vector(0, 0, 1), r), math.pi / 2, math.pi
        ).toShape(),
        Part.LineSegment(App.Vector(x0, y1 - r, 0), App.Vector(x0, y0 + r, 0)).toShape(),
        Part.ArcOfCircle(
            Part.Circle(App.Vector(x0 + r, y0 + r, 0), App.Vector(0, 0, 1), r), math.pi, 3 * math.pi / 2
        ).toShape(),
    ]
    wire = Part.Wire(edges)
    return wire


def bean_wire(rx=7.0, ry=4.5):
    return Part.Ellipse(App.Vector(0, 0, 0), rx, ry).toShape()


# Main teardrop bulge radius (plan); shoulder blends are tangent arcs of DROP_SHOULDER_R.
DROP_MAIN_R = 6.0
DROP_SHOULDER_R = 3.0
DROP_TIP_R = 2.0
DROP_TIP_Y = -5.0
DROP_TIP_SPREAD = 0.45


def _shoulder_fillet_center(R, sr, V, d, side):
    """Center of a sr fillet tangent to main circle (R) and the side line through V."""
    d = unit(d)
    target = R - sr
    best = None
    for i in range(3600):
        ang = math.radians(i / 10.0)
        C = App.Vector(target * math.cos(ang), target * math.sin(ang), 0)
        if C.y < 0.3:
            continue
        if side == "right" and C.x > V.x - 0.2:
            continue
        if side == "left" and C.x < V.x + 0.2:
            continue
        w = C - V
        perp = w - d * (w.dot(d))
        dd = abs(perp.Length - sr)
        if best is None or dd < best[0]:
            best = (dd, C)
    if best is None or best[0] > 0.02:
        raise RuntimeError(f"drop shoulder fillet failed ({side})")
    return best[1]


def _shoulder_fillet_points(R, sr, V, d, side):
    C = _shoulder_fillet_center(R, sr, V, d, side)
    Pc = unit(C) * R
    d = unit(d)
    w = C - V
    t = w.dot(d)
    h = (w - d * t).Length
    off = math.sqrt(max(0.0, sr * sr - h * h))
    Pl = V + d * max(0.0, t - off)
    return Pc, Pl, C


def _fillet_arc(center, radius, p_from, p_to):
    circ = Part.Circle(center, App.Vector(0, 0, 1), radius)
    a1 = math.atan2(p_from.y - center.y, p_from.x - center.x)
    a2 = math.atan2(p_to.y - center.y, p_to.x - center.x)
    return Part.ArcOfCircle(circ, a1, a2).toShape()


def drop_wire():
    """Teardrop: dome on +Y, blunt tip on -Y; cable enters +X; round shoulder fillets."""
    r = DROP_MAIN_R
    sr = DROP_SHOULDER_R
    tip_r = DROP_TIP_R
    tip_y = DROP_TIP_Y
    t_side = DROP_TIP_SPREAD
    tr = -math.pi / 2 + t_side
    tl = -math.pi / 2 - t_side
    pr = App.Vector(tip_r * math.cos(tr), tip_y + tip_r * math.sin(tr), 0)
    pl = App.Vector(tip_r * math.cos(tl), tip_y + tip_r * math.sin(tl), 0)
    Vr = App.Vector(r, 0, 0)
    Vl = App.Vector(-r, 0, 0)
    Pcr, Plr, Cr = _shoulder_fillet_points(r, sr, Vr, pr - Vr, "right")
    Pcl, Pll, Cl = _shoulder_fillet_points(r, sr, Vl, pl - Vl, "left")
    dome = Part.ArcOfCircle(
        Part.Circle(App.Vector(0, 0, 0), App.Vector(0, 0, 1), r),
        math.atan2(Pcr.y, Pcr.x),
        math.atan2(Pcl.y, Pcl.x),
    ).toShape()
    right = Part.LineSegment(pr, Plr).toShape()
    left = Part.LineSegment(Pll, pl).toShape()
    tip = Part.ArcOfCircle(
        Part.Circle(App.Vector(0, tip_y, 0), App.Vector(0, 0, 1), tip_r), tl, tr
    ).toShape()
    arc_r = _fillet_arc(Cr, sr, Plr, Pcr)
    arc_l = _fillet_arc(Cl, sr, Pcl, Pll)
    wire = Part.Wire([right, arc_r, dome, arc_l, left, tip])
    if not wire.isClosed():
        raise RuntimeError("drop outline not closed")
    face = Part.Face(wire)
    if not face.isValid():
        raise RuntimeError("drop profile face invalid")
    return wire


def extrude_profile(wire, thick):
    face = Part.Face(wire)
    if not face.isValid():
        raise RuntimeError("invalid profile face")
    return Part.Face(wire).extrude(App.Vector(0, 0, -thick))


def build_hex(thick, bolt_r):
    wire = hex_wire(12.0)
    solid = extrude_profile(wire, thick)
    return side_top_pinch(solid, thick, bolt_r, face_x=6.0, face_y=0.0)


def build_squircle(thick, bolt_r):
    wire = squircle_wire(12.0, 3.0, 0.0)
    solid = extrude_profile(wire, thick)
    solid, mouth, screw_d = side_top_pinch(solid, thick, bolt_r, face_x=6.0, face_y=0.0)
    zmid = -thick / 2.0
    mouth_xy = App.Vector(mouth.x, mouth.y, 0)
    solid.rotate(App.Vector(0, 0, 0), App.Vector(0, 0, 1), 8.0)
    mouth = rot_z(mouth_xy, 8.0)
    mouth.z = zmid
    return solid, mouth, screw_d


def build_drop(thick, bolt_r):
    wire = drop_wire()
    solid = extrude_profile(wire, thick)
    solid = outer_fillet(solid, thick)
    zmid = -thick / 2.0
    mouth_xy = positive_x_mouth_on_solid(solid, zmid, prefer_flat_center=False)
    return side_top_pinch(
        solid, thick, bolt_r, verify=False, mouth_override=mouth_xy, already_filleted=True
    )


def build_bean(thick, bolt_r):
    edge = bean_wire(7.0, 4.5)
    wire = Part.Wire([edge])
    solid = extrude_profile(wire, thick)
    return barrel_pinch(solid, thick, bolt_r, 7.0, 0.0)


def mesh_check(solid, name):
    mesh = MeshPart.meshFromShape(Shape=solid, LinearDeflection=0.008, AngularDeflection=0.12)
    mesh_topology_check(mesh, name)
    bb = mesh.BoundBox
    log(name, "mesh", f"{bb.XLength:.2f}x{bb.YLength:.2f}x{bb.ZLength:.2f}", "ymin", round(bb.YMin, 3))
    return mesh


def check_solid(solid, thick, bolt_r, mouth, screw_depth, pinch):
    if len(solid.Solids) != 1:
        raise RuntimeError("not one solid")
    zmid = -thick / 2.0
    if mouth.Length > 1e-6:
        inward_xy = App.Vector(-mouth.x, -mouth.y, 0)
        if inward_xy.Length > 1e-6:
            inward_xy.normalize()
        else:
            inward_xy = App.Vector(-1, 0, 0)
    else:
        inward_xy = App.Vector(-1, 0, 0)
    tests = [
        (App.Vector(0, 0, zmid), False),
        (App.Vector(0, 0, -screw_depth + 0.2), False),
        (App.Vector(0, 0, -thick + 0.35), True),
        (App.Vector(0, 0, -0.35), False),
        (App.Vector(mouth.x, mouth.y, zmid) + inward_xy * (-0.4), False),
    ]
    for p, want in tests:
        got = solid.isInside(p, 1e-5, True)
        if got != want:
            raise RuntimeError(f"inside {p} got {got} want {want} ({pinch})")


BUILDERS = {
    "hex": build_hex,
    "bean": build_bean,
    "drop": build_drop,
    "squircle": build_squircle,
}

PLACEMENTS = {
    "hex": ((66, 8, 0), (66, 8, 17)),
    "drop": ((82, 8, 0), (82, 8, 17)),
    "squircle": ((98, 8, 0), (98, 8, 17)),
    "bean": ((18, 22, 0), (18, 22, 17)),
}

shapes = {}
try:
    for style, fn in BUILDERS.items():
        for suffix, thick, bolt_r in (("m3", 5.0, 1.45), ("m4", 7.0, 1.9)):
            log("build", style, suffix)
            solid, cable_c, screw_d = fn(thick, bolt_r)
            check_solid(solid, thick, bolt_r, cable_c, screw_d, style)
            mesh_check(solid, f"{style}-{suffix}")
            shapes[f"{style}-{suffix}"] = solid
except Exception:
    open("/tmp/tier_a_err.txt", "w").write(traceback.format_exc())
    log("BUILD FAILED")
    raise SystemExit(1)

doc = App.openDocument(DOC_PATH)
for label, vol in KEEP.items():
    if vol is None:
        o = doc.getObject(label.replace("-", "_").replace("heart", "HeartShape"))
        if o is None:
            for obj in doc.Objects:
                if obj.Label == label:
                    o = obj
                    break
        if o is None:
            continue
        vol = round(o.Shape.Volume, 3)
        KEEP[label] = vol
    else:
        o = None
        for obj in doc.Objects:
            if obj.Label == label:
                o = obj
                break
        if o is None:
            raise RuntimeError(f"missing {label}")
        if abs(o.Shape.Volume - vol) > 0.05:
            raise RuntimeError(f"drift {label} {o.Shape.Volume} != {vol}")

protected = [o.Name for o in doc.Objects]


def store(feat, shape, origin):
    feat.Placement = App.Placement()
    shp = shape.copy()
    shp.Placement = App.Placement()
    feat.Shape = shp
    feat.Placement = App.Placement(App.Vector(*origin), App.Rotation())


def ensure_feature(internal, label):
    obj = doc.getObject(internal)
    if obj is None:
        obj = doc.addObject("Part::Feature", internal)
        obj.Label = label
    return obj


def ensure_group(internal, label):
    grp = doc.getObject(internal)
    if grp is None:
        grp = doc.addObject("App::DocumentObjectGroup", internal)
        grp.Label = label
    return grp


for style in BUILDERS:
    grp = ensure_group(f"{style.capitalize()}Group", style)
    for suffix in ("m3", "m4"):
        internal = f"{style.capitalize()}Shape_{suffix}"
        label = f"{style}-{suffix}"
        feat = ensure_feature(internal, label)
        origin = PLACEMENTS[style][0 if suffix == "m3" else 1]
        store(feat, shapes[label], origin)
        if feat not in grp.Group:
            grp.addObject(feat)

sheet = doc.getObject("Spreadsheet")
if sheet:
    aliases = {
        "HexFlatToFlat": "12 mm",
        "BeanLength": "14 mm",
        "BeanWidth": "9 mm",
        "DropHeight": "12 mm",
        "DropShoulderRadius": "3 mm",
        "SquircleSize": "12 mm",
        "SquircleTilt": "8 deg",
    }
    row = 20
    for alias, val in aliases.items():
        try:
            sheet.setAlias(f"A{row}", alias)
        except Exception:
            pass
        sheet.set(f"A{row}", val.split()[0])
        sheet.set(f"B{row}", val)
        row += 1

for name in protected:
    obj = doc.getObject(name)
    if obj and not obj.Name.startswith(("Hex", "Bean", "Drop", "Squircle")):
        try:
            obj.purgeTouched()
        except Exception:
            pass

for label, vol in KEEP.items():
    if vol is None:
        continue
    o = next((x for x in doc.Objects if x.Label == label), None)
    if o and abs(o.Shape.Volume - vol) > 0.05:
        raise RuntimeError(f"changed legacy {label}")

doc.save()

for style in BUILDERS:
    for suffix in ("m3", "m4"):
        name = f"mon-cheri-{style}-{suffix}.stl"
        feat = doc.getObject(f"{style.capitalize()}Shape_{suffix}")
        mesh = MeshPart.meshFromShape(Shape=feat.Shape, LinearDeflection=0.008, AngularDeflection=0.12)
        mesh.write(f"{REPO}/{name}")
        log("wrote", name)

log("TIER_A_OK")
