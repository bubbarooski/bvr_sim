"""Where an aircraft type's files are, for the model-fitting tools.

A type is data/aircraft/<id>/aircraft.json (see docs/SCHEMA.md, Aircraft
types). Its flight model is data/jsbsim_data/aircraft/<model>/<model>.xml,
whose <engine file="..."> names the engine file the thrust fit rewrites.
Published data the type is fitted and validated against lives in
reference/; tool settings and generated artifacts in fit/.

Paths are relative to the repo root, which the tools run from.
"""

import json
import os
import re

AIRCRAFT_ROOT = "data/aircraft"
JSBSIM_ROOT = "data/jsbsim_data"


class AircraftData:
    def __init__(self, type_id):
        self.id = type_id
        self.directory = os.path.join(AIRCRAFT_ROOT, type_id)
        path = os.path.join(self.directory, "aircraft.json")
        if not os.path.exists(path):
            raise SystemExit("no aircraft type '%s' -- expected %s" % (type_id, path))
        with open(path, encoding="utf-8") as f:
            self.type = json.load(f)
        model = self.type["jsbsim_model"]
        self.model_xml = os.path.join(JSBSIM_ROOT, "aircraft", model, model + ".xml")
        with open(self.model_xml, encoding="utf-8") as f:
            engines = re.findall(r'<engine\s+file="([^"]+)"', f.read())
        if len(engines) != 1:
            raise SystemExit("%s: expected one <engine file=...>, found %d" % (self.model_xml, len(engines)))
        self.engine_xml = os.path.join(JSBSIM_ROOT, "engine", engines[0] + ".xml")

    def reference(self, name):
        return os.path.join(self.directory, "reference", name)

    def fit(self, name):
        return os.path.join(self.directory, "fit", name)

    def fit_settings(self, name):
        with open(self.fit(name), encoding="utf-8") as f:
            return json.load(f)


def add_aircraft_argument(parser):
    parser.add_argument("--aircraft", required=True, help="aircraft type id under data/aircraft (e.g. f16c_block50)")
