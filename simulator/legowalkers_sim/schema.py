"""Versioned interchange contract. JSON is validated before simulator access."""

from copy import deepcopy


def record(fields, *, optional=()):
    return {
        "type": "object",
        "properties": fields,
        "required": [name for name in fields if name not in optional],
        "additionalProperties": False,
    }


def array(item):
    return {"type": "array", "items": item}


TEXT = {"type": "string", "minLength": 1}
NUMBER = {"type": "number"}
INTEGER = {"type": "integer"}
VECTOR = {"type": "array", "items": NUMBER, "minItems": 3, "maxItems": 3}
ROTATION = {"type": "array", "items": VECTOR, "minItems": 3, "maxItems": 3}
JOINT = record({
    "id": TEXT, "bodyA": TEXT, "bodyB": TEXT, "kind": {"const": "revolute"},
    "position": VECTOR, "axis": VECTOR, "connector": TEXT,
})
MANIFEST_SCHEMA = record({
    "schemaVersion": {"const": 1},
    "grammarVersion": {"enum": ["technic-fourbar-v1", "technic-fourbar-v2", "technic-fourbar-v3", "technic-fourbar-v4", "technic-fourbar-v5"]},
    "modelKind": {"enum": ["walker", "leg", "hinge", "rigid", "motor"]},
    "modelFile": TEXT,
    "ldrawSha256": {"type": "string", "pattern": "^[0-9a-f]{64}$"},
    "simulator": record({"repository": TEXT, "revision": TEXT}),
    "units": record({
        "length": {"const": "ldu"}, "lduToMeters": {"const": 0.0004},
        "up": {"const": [0, -1, 0]}, "forward": {"const": [1, 0, 0]},
    }),
    "genome": record({
        "pairs": {"type": "integer", "minimum": 1, "maximum": 3},
        "groundHoles": {"enum": [3, 4]}, "couplerHoles": {"enum": [3, 4]},
        "rockerHoles": {"enum": [3, 4]},
        "phaseQuarterTurns": {"type": "integer", "minimum": 0, "maximum": 3},
        "seed": INTEGER,
        "remote": {"type": "boolean"},
        "batterySlot": {"enum": [0, 1, 2]}, "batteryLevel": {"enum": [0, 1]},
        "receiverSlot": {"enum": [0, 1, 2]}, "receiverSide": {"enum": [0, 1]},
        "motorDirection": {"enum": [-1, 1]},
        "towWagon": {"type": "boolean"}, "batteryOnWagon": {"type": "boolean"},
        "receiverOnWagon": {"type": "boolean"}, "wagonAxleHole": {"enum": [2, 3, 4]},
    }, optional=("remote", "batterySlot", "batteryLevel", "receiverSlot", "receiverSide", "motorDirection",
                 "towWagon", "batteryOnWagon", "receiverOnWagon", "wagonAxleHole")),
    "payloads": array(record({"partInstance": TEXT, "product": {"enum": ["8881", "8884"]},
                              "carrier": {"enum": ["walker", "wagon"]}}, optional=("carrier",))),
    "groundWheels": array(TEXT),
    "chassis": TEXT,
    "parts": array(record({
        "id": TEXT, "part": TEXT, "color": INTEGER, "position": VECTOR,
        "rotation": ROTATION, "body": {"type": ["string", "null"]},
    })),
    "expected": record({
        "bodies": array(TEXT), "joints": array(JOINT),
        "gears": array(record({"bodyA": TEXT, "bodyB": TEXT, "ratio": {"type": "number", "exclusiveMinimum": 0}})),
        "drives": array(record({
            "motorPart": TEXT, "joint": TEXT, "speed": NUMBER,
            "maxTorque": {"type": "number", "exclusiveMinimum": 0},
        })),
    }),
    "feet": array(record({"id": TEXT, "body": TEXT, "position": VECTOR, "partInstance": TEXT,
                          "contactRadiusMeters": {"type": "number", "exclusiveMinimum": 0, "maximum": .004}},
                         optional=("partInstance", "contactRadiusMeters"))),
    "footPaths": array(record({
        "id": TEXT, "strideLdu": NUMBER, "liftLdu": NUMBER, "points": array(VECTOR),
    })),
    "warnings": array(TEXT),
}, optional=("payloads", "groundWheels"))

LEGACY_MANIFEST_SCHEMA = MANIFEST_SCHEMA
HASH = {"type": "string", "pattern": "^[0-9a-f]{64}$"}
MOTION_MANIFEST_SCHEMA = deepcopy(LEGACY_MANIFEST_SCHEMA)
MOTION_MANIFEST_SCHEMA["properties"].update({
    "grammarVersion": {"const": "technic-motion-v1"},
    "modelKind": {"const": "leg"},
    "genome": record({
        "schemaVersion": {"const": 1}, "recipeVersion": {"const": "planar-parts-v1"},
        "mechanismId": HASH, "side": {"enum": [-1, 1]},
        "bodies": array(record({
            "body": TEXT, "part": TEXT, "layer": {"type": "integer", "minimum": 0, "maximum": 16},
            "offset": NUMBER,
        })),
        "connections": array(record({"group": TEXT, "part": TEXT, "center": NUMBER})),
    }),
    "motion": record({
        "mechanismId": HASH, "genotypeId": HASH, "assemblyId": HASH,
        "gateVersion": {"const": "motion-gates-v1"},
        "cycleSteps": {"type": "integer", "minimum": 36, "maximum": 3600},
        "sharedPivots": array(record({"id": TEXT, "abstractHinges": array(TEXT), "bodies": array(TEXT)})),
    }),
    "payloads": {"const": []},
    "groundWheels": {"const": []},
})
MOTION_MANIFEST_SCHEMA["required"].extend(["motion", "payloads", "groundWheels"])
MIN_WALKER_FEET = 2
MAX_FEET = 6
MOTION_V2_MANIFEST_SCHEMA = deepcopy(MOTION_MANIFEST_SCHEMA)
MOTION_V2_MANIFEST_SCHEMA["properties"]["grammarVersion"] = {"const": "technic-motion-v2"}
motion = MOTION_V2_MANIFEST_SCHEMA["properties"]["motion"]
motion["properties"].update({
    "footCount": {"type": "integer", "minimum": 1, "maximum": MAX_FEET},
    "logicalFeet": {**array(record({"id": TEXT, "body": TEXT, "regions": array(TEXT)})),
                    "minItems": 1, "maxItems": MAX_FEET},
})
motion["required"].extend(["footCount", "logicalFeet"])
feet = MOTION_V2_MANIFEST_SCHEMA["properties"]["feet"]["items"]
feet["properties"]["logicalFoot"] = TEXT
feet["required"].append("logicalFoot")
POINT_RECORD = record({"x": NUMBER, "y": NUMBER, "z": NUMBER})
PLACEMENT = record({
    "id": TEXT, "part": TEXT, "color": INTEGER, "position": POINT_RECORD,
    "rotation": record({"x": POINT_RECORD, "y": POINT_RECORD, "z": POINT_RECORD}),
})
WALKER_MANIFEST_SCHEMA = deepcopy(MOTION_V2_MANIFEST_SCHEMA)
WALKER_MANIFEST_SCHEMA["properties"].update({
    "grammarVersion": {"const": "technic-motion-v3"},
    "modelKind": {"const": "walker"},
    "genome": record({
        "schemaVersion": {"const": 1}, "recipeVersion": {"const": "walker-parts-v1"}, "mechanismId": HASH,
        "remote": {"const": True}, "towWagon": {"const": False},
        "bodies": array(record({"body": TEXT, "parts": {**array(PLACEMENT), "minItems": 1}})),
        "connections": array(record({
            "group": TEXT, "hinges": {**array(TEXT), "minItems": 1}, "placement": PLACEMENT,
        })),
        "mounts": array(PLACEMENT),
        "feet": array(record({"foot": TEXT, "part": TEXT, "depth": NUMBER})),
        "motorPart": TEXT, "speed": NUMBER, "maxTorque": {"type": "number", "exclusiveMinimum": 0},
    }),
    "payloads": {**deepcopy(LEGACY_MANIFEST_SCHEMA["properties"]["payloads"]), "minItems": 2, "maxItems": 2},
})
WALKER_MANIFEST_SCHEMA["properties"]["motion"]["properties"]["footCount"]["minimum"] = MIN_WALKER_FEET
WALKER_MANIFEST_SCHEMA["properties"]["motion"]["properties"]["logicalFeet"]["minItems"] = MIN_WALKER_FEET
WALKER_MANIFEST_SCHEMA["properties"]["motion"]["properties"]["logicalFeet"]["items"]["properties"]["contactShape"] = {
    "enum": ["capsule", "rectangle"]}
WALKER_MANIFEST_SCHEMA["properties"]["motion"]["properties"]["logicalFeet"]["items"]["properties"]["surfaces"] = {
    **array(record({"contactShape": {"enum": ["capsule", "rectangle"]}, "regions": array(TEXT), "body": TEXT}, optional=("body",))),
    "minItems": 1, "maxItems": 2,
}
WALKER_MANIFEST_SCHEMA["properties"]["genome"]["properties"]["feet"]["items"]["properties"]["additionalContacts"] = {
    **array(record({"part": TEXT, "depth": NUMBER})), "minItems": 1, "maxItems": 1,
}
WALKER_MANIFEST_SCHEMA["properties"]["genome"]["properties"]["legFamilies"] = array(record({
    "id": TEXT,
    "instances": {**array(record({
        "id": TEXT, "foot": TEXT, "mirrored": {"type": "boolean"}, "phase": NUMBER,
        "bodies": {**array(record({"role": TEXT, "body": TEXT})), "minItems": 1},
        "attachments": {**array(record({
            "role": TEXT, "port": record({"body": TEXT, "site": TEXT}),
        })), "minItems": 1},
    })), "minItems": 1},
}))
MANIFEST_SCHEMA = {"oneOf": [
    LEGACY_MANIFEST_SCHEMA, MOTION_MANIFEST_SCHEMA, MOTION_V2_MANIFEST_SCHEMA, WALKER_MANIFEST_SCHEMA,
]}
