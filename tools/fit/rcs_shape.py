"""Estimate an aircraft type's radar cross-section shape from its geometry.

ESTIMATE, not data: no measured RCS of any aircraft type in this repo is
public. The type's mean RCS seen from a direction is taken to be the
aircraft's projected (silhouette) area in that direction -- the
optical-region RCS of a large sphere is exactly its projected area pi r^2,
and for a target built of many comparable scatterers (the Swerling 1
picture the detector already assumes) the visible scatterer count grows
with the area the radar sees. The airframe is four ellipsoids aligned with
the body axes, sized from the repo's own geometry:

  fuselage    length = the type's hit box; depth = intake lower lip to the
              pilot's eyepoint; width = between the ventral fins and the
              main gear track (JSBSim f16.xml structure points)
  wing        span and area (f16.xml metrics); an ellipse of that span and
              area; thickness THICKNESS_RATIO x the mean aerodynamic chord
  horizontal  area (f16.xml metrics); span from HTAIL_ASPECT_RATIO
  tail
  fin         area (f16.xml metrics); height = fin tip (TOP_VS) to the
              eyepoint level

Projected area of an ellipsoid with semi-axes (s, h, l) along unit view
direction n = (n_span, n_up, n_nose) in body axes:

  A(n) = pi sqrt((h l n_span)^2 + (s l n_up)^2 + (s h n_nose)^2)

and the airframe's is the sum over its four ellipsoids (overlap ignored:
the wing root inside the fuselage is counted twice from above). The model
has no specular flashes, cavities (inlet, nozzle), edge diffraction or
frequency dependence; real fighters are spikier than a silhouette.

    python tools/fit/rcs_shape.py f16c_block50           # report
    python tools/fit/rcs_shape.py f16c_block50 --write   # store rcs_shape

Writes signature.rcs_shape in data/aircraft/<id>/aircraft.json:
[{"part": ..., "semi_axes_ft": {"span": s, "height": h, "length": l}}].
"""

import argparse
import json
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
FT2_TO_M2 = 0.3048 ** 2

# ESTIMATES with no source (docs/REFERENCE.md 9.2).
THICKNESS_RATIO = 0.04     # wing, tail and fin thickness / chord: a thin fighter section
HTAIL_ASPECT_RATIO = 3.0   # horizontal tail span^2 / area, taken equal to the wing's (30^2 / 300)


def jsbsim_geometry(model):
    root = ET.parse(REPO / "data" / "jsbsim_data" / "aircraft" / model / f"{model}.xml").getroot()
    metrics = root.find("metrics")
    num = lambda tag: float(metrics.find(tag).text)
    points = {}
    for el in list(metrics.findall("location")) + list(root.iter("contact")):
        loc = el.find("location") if el.tag == "contact" else el
        points[el.get("name")] = {axis: float(loc.find(axis).text) for axis in ("x", "y", "z")}
    return {"wing_area": num("wingarea"), "span": num("wingspan"), "chord": num("chord"),
            "htail_area": num("htailarea"), "vtail_area": num("vtailarea"), "points": points}


def shape(geometry, hit_box):
    """Four body-axis ellipsoids, semi-axes in ft."""
    p = geometry["points"]
    eye_z = p["EYEPOINT"]["z"]
    depth_ft = (eye_z - p["INTAKE"]["z"]) / 12.0
    half_width_ft = 0.5 * (abs(p["RIGHT_VFT"]["y"]) + abs(p["RIGHT_MLG"]["y"])) / 12.0

    wing_s = geometry["span"] / 2.0
    wing_l = geometry["wing_area"] / (math.pi * wing_s)
    wing_h = THICKNESS_RATIO * geometry["chord"] / 2.0

    h_span = math.sqrt(HTAIL_ASPECT_RATIO * geometry["htail_area"])
    h_s = h_span / 2.0
    h_l = geometry["htail_area"] / (math.pi * h_s)
    h_h = THICKNESS_RATIO * (geometry["htail_area"] / h_span) / 2.0

    fin_height = (p["TOP_VS"]["z"] - eye_z) / 12.0
    v_h = fin_height / 2.0
    v_l = geometry["vtail_area"] / (math.pi * v_h)
    v_s = THICKNESS_RATIO * (geometry["vtail_area"] / fin_height) / 2.0

    parts = [("fuselage", half_width_ft, depth_ft / 2.0, hit_box["length"] / 2.0),
             ("wing", wing_s, wing_h, wing_l),
             ("horizontal_tail", h_s, h_h, h_l),
             ("fin", v_s, v_h, v_l)]
    return [{"part": name, "semi_axes_ft": {"span": round(s, 4), "height": round(h, 4), "length": round(l, 4)}}
            for name, s, h, l in parts]


def projected_area_m2(parts, n_span, n_up, n_nose):
    total = 0.0
    for part in parts:
        a = part["semi_axes_ft"]
        s, h, l = a["span"], a["height"], a["length"]
        total += math.pi * math.sqrt((h * l * n_span) ** 2 + (s * l * n_up) ** 2 + (s * h * n_nose) ** 2)
    return total * FT2_TO_M2


def view(off_nose_deg, elevation_deg):
    """Direction from the aircraft to the radar: off_nose in the wing plane, elevation above it."""
    az, el = math.radians(off_nose_deg), math.radians(elevation_deg)
    return math.cos(el) * math.sin(az), math.sin(el), math.cos(el) * math.cos(az)


def format_json(obj):
    """Indent like the hand-written type files, but keep arrays of numbers on one line
    (the same formatter as tools/fit/fit_normal_force.py)."""
    text = json.dumps(obj, indent=4, ensure_ascii=False)
    number = r"-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?"
    pattern = re.compile(r"\[\s*(%s(?:\s*,\s*%s)*)\s*\]" % (number, number))
    return pattern.sub(lambda m: "[" + ", ".join(re.split(r"\s*,\s*", m.group(1).strip())) + "]", text) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("aircraft", help="aircraft type id (data/aircraft/<id>)")
    parser.add_argument("--write", action="store_true", help="store rcs_shape in the type file")
    args = parser.parse_args()

    path = REPO / "data" / "aircraft" / args.aircraft / "aircraft.json"
    doc = json.loads(path.read_text(encoding="utf-8"))
    parts = shape(jsbsim_geometry(doc["jsbsim_model"]), doc["geometry"]["hit_box_ft"])
    for part in parts:
        a = part["semi_axes_ft"]
        print("%-16s semi-axes ft: span %.3f  height %.3f  length %.3f" % (part["part"], a["span"], a["height"], a["length"]))
    print("mean RCS estimate (m2), radar in the wing plane:")
    for off in (0, 30, 60, 90, 120, 150, 180):
        print("  %3d deg off the nose: %6.2f" % (off, projected_area_m2(parts, *view(off, 0))))
    print("  from above / below:   %6.2f" % projected_area_m2(parts, *view(0, 90)))

    if args.write:
        # Only the signature block's text is replaced: a whole-file dump
        # would respell hand-written numbers (9.86e9, 1e-6) elsewhere.
        signature = dict(doc["signature"])
        signature["rcs_shape"] = parts
        block = format_json({"signature": signature}).strip()[1:-1].strip("\n")
        block = "\n".join(line[4:] if line.startswith("    ") else line for line in block.split("\n"))
        block = "\n".join("    " + line for line in block.split("\n"))
        text = path.read_text(encoding="utf-8")
        new_text, count = re.subn(r'    "signature": \{.*?\n    \}', lambda m: block, text, count=1, flags=re.S)
        if count != 1:
            raise SystemExit("no signature block found in %s" % path.relative_to(REPO))
        path.write_text(new_text, encoding="utf-8", newline="\n")
        print("wrote %s" % path.relative_to(REPO))


if __name__ == "__main__":
    main()
