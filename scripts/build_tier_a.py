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


def lip_fillet(solid, cable_center, screw_center, bolt_r, zmid, mouth_x_min=0.0):
    cable_e, screw_e = [], []
    for e in solid.Edges:
        c = curve_of(e)
        if c is None or type(c).__name__ != "Circle":
            continue
        cc = c.Center
        if abs(c.Radius - CABLE_R) < 0.05 and abs(cc.z - zmid) < 0.15:
            if cc.x >= mouth_x_min and (App.Vector(cc.x, cc.y, 0) - App.Vector(cable_center.x, cable_center.y, 0)).Length < 1.2:
                cable_e.append(e)
        if abs(c.Radius - bolt_r) < 0.06 and abs(cc.x - screw_center.x) < 0.2 and abs(cc.y - screw_center.y) < 0.2:
            if abs(cc.z) < 0.15 and abs(e.Length - 2 * math.pi * bolt_r) < 0.15:
                screw_e.append(e)
    if not cable_e or len(screw_e) != 1:
        raise RuntimeError(f"lip edges cable={len(cable_e)} screw={len(screw_e)}")
    cable_e.sort(key=lambda e: -e.Length)
    try:
        solid = solid.makeFillet(LIP_R, screw_e)
        solid = solid.makeFillet(LIP_R, [cable_e[0]])
    except Exception:
        pass
    try:
        solid = solid.removeSplitter()
    except Exception:
        pass
    return solid


def square_pinch(solid, thick, bolt_r, half_x, half_y=None):
    """Cable +X face, screw down through top center (0,0). Profile centered on origin."""
    if half_y is None:
        half_y = half_x
    solid = vertical_fillet(solid, thick)
    zmid = -thick / 2.0
    screw_depth = thick - 1.0
    cable_center = App.Vector(half_x, 0, zmid)
    dist = half_x
    cable_depth = dist + bolt_r + 0.45
    if cable_depth + CONE_H > 2 * dist - 1.0:
        raise RuntimeError("cable breaks out (square pinch)")
    start = App.Vector(half_x, 0, zmid) - App.Vector(1, 0, 0) * 0.25
    solid = solid.cut(Part.makeCylinder(CABLE_R, cable_depth + 0.25, start, App.Vector(-1, 0, 0)))
    cone_at = App.Vector(half_x, 0, zmid) + App.Vector(-1, 0, 0) * cable_depth
    solid = solid.cut(Part.makeCone(CABLE_R, 0.0, CONE_H, cone_at, App.Vector(-1, 0, 0)))
    solid = solid.cut(
        Part.makeCylinder(
            bolt_r,
            screw_depth + 0.25,
            App.Vector(0, 0, 0.25),
            App.Vector(0, 0, -1),
        )
    )
    solid = lip_fillet(
        solid,
        cable_center,
        App.Vector(0, 0, 0),
        bolt_r,
        zmid,
        mouth_x_min=half_x - 1.5,
    )
    return solid, cable_center, screw_depth


def barrel_pinch(solid, thick, bolt_r, surface_x, surface_y=0.0):
    """Cable enters +X side at (surface_x, surface_y), screw through top center."""
    solid = vertical_fillet(solid, thick)
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
    solid = lip_fillet(
        solid,
        cable_center,
        App.Vector(0, 0, 0),
        bolt_r,
        zmid,
        mouth_x_min=surface_x - 1.5,
    )
    return solid, cable_center, screw_depth


def hex_wire(flat_to_flat=12.0):
    a = flat_to_flat / 2.0
    h = a * math.sqrt(3)
    pts = [
        App.Vector(a, 0, 0),
        App.Vector(a / 2, h / 2, 0),
        App.Vector(-a / 2, h / 2, 0),
        App.Vector(-a, 0, 0),
        App.Vector(-a / 2, -h / 2, 0),
        App.Vector(a / 2, -h / 2, 0),
    ]
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
    if abs(tilt_deg) > 0.01:
        wire.rotate(App.Vector(0, 0, 0), App.Vector(0, 0, 1), tilt_deg)
    return wire


def bean_wire(rx=7.0, ry=4.5):
    return Part.Ellipse(App.Vector(0, 0, 0), rx, ry).toShape()


def drop_wire():
    """Teardrop: dome on +Y side of circle, blunt tip on -Y; cable enters +X."""
    r = 6.0
    tip_r = 2.0
    tip_y = -5.0
    t_side = 0.45
    tr = -math.pi / 2 + t_side
    tl = -math.pi / 2 - t_side
    pr = App.Vector(tip_r * math.cos(tr), tip_y + tip_r * math.sin(tr), 0)
    pl = App.Vector(tip_r * math.cos(tl), tip_y + tip_r * math.sin(tl), 0)
    dome = Part.ArcOfCircle(
        Part.Circle(App.Vector(0, 0, 0), App.Vector(0, 0, 1), r), 0, math.pi
    ).toShape()
    right = Part.LineSegment(App.Vector(r, 0, 0), pr).toShape()
    tip = Part.ArcOfCircle(
        Part.Circle(App.Vector(0, tip_y, 0), App.Vector(0, 0, 1), tip_r), tl, tr
    ).toShape()
    left = Part.LineSegment(pl, App.Vector(-r, 0, 0)).toShape()
    wire = Part.Wire(Part.sortEdges([dome, right, tip, left])[0])
    if not wire.isClosed():
        raise RuntimeError("drop outline not closed")
    return wire


def extrude_profile(wire, thick):
    face = Part.Face(wire)
    if not face.isValid():
        raise RuntimeError("invalid profile face")
    return Part.Face(wire).extrude(App.Vector(0, 0, -thick))


def build_hex(thick, bolt_r):
    wire = hex_wire(12.0)
    solid = extrude_profile(wire, thick)
    return square_pinch(solid, thick, bolt_r, 6.0, 6.0 * math.sqrt(3) / 2)


def build_squircle(thick, bolt_r):
    wire = squircle_wire(12.0, 3.0, 8.0)
    bb = wire.BoundBox
    half_x = bb.XMax
    solid = extrude_profile(wire, thick)
    return square_pinch(solid, thick, bolt_r, half_x, max(abs(bb.YMin), bb.YMax))


def build_drop(thick, bolt_r):
    wire = drop_wire()
    bb = wire.BoundBox
    solid = extrude_profile(wire, thick)
    return square_pinch(solid, thick, bolt_r, bb.XMax, max(abs(bb.YMin), bb.YMax))


def build_bean(thick, bolt_r):
    edge = bean_wire(7.0, 4.5)
    wire = Part.Wire([edge])
    solid = extrude_profile(wire, thick)
    return barrel_pinch(solid, thick, bolt_r, 7.0, 0.0)


def mesh_check(solid, name):
    mesh = MeshPart.meshFromShape(Shape=solid, LinearDeflection=0.008, AngularDeflection=0.12)
    if mesh.hasNonManifolds():
        raise RuntimeError(f"{name} non-manifold mesh")
    bb = mesh.BoundBox
    log(name, "mesh", f"{bb.XLength:.2f}x{bb.YLength:.2f}x{bb.ZLength:.2f}", "ymin", round(bb.YMin, 3))
    return mesh


def check_solid(solid, thick, bolt_r, cable_center, screw_depth, pinch):
    if len(solid.Solids) != 1:
        raise RuntimeError("not one solid")
    zmid = -thick / 2.0
    tests = [
        (App.Vector(0, 0, zmid), False),
        (App.Vector(0, 0, -screw_depth + 0.2), False),
        (App.Vector(0, 0, -thick + 0.35), True),
        (App.Vector(0, 0, -0.35), False),
        (cable_center + App.Vector(-0.4, 0, 0), False),
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
