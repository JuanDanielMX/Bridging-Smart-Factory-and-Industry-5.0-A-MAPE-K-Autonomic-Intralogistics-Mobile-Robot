#!/usr/bin/env python3
"""Static validation that does not require a ROS installation."""

import ast
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "catkin_ws" / "src" / "autonomic_turtlebot3"


def validate() -> None:
    errors = []

    for path in [PACKAGE / "package.xml", PACKAGE / "rrt_global_planner_plugin.xml"]:
        try:
            ET.parse(path)
        except Exception as exc:
            errors.append(f"{path.relative_to(ROOT)}: {exc}")

    for path in sorted((PACKAGE / "launch").glob("*.launch")):
        try:
            ET.parse(path)
        except Exception as exc:
            errors.append(f"{path.relative_to(ROOT)}: {exc}")

    for path in sorted(PACKAGE.rglob("*.yaml")):
        try:
            yaml.safe_load(path.read_text(encoding="utf-8"))
        except Exception as exc:
            errors.append(f"{path.relative_to(ROOT)}: {exc}")

    for path in sorted(PACKAGE.rglob("*.py")) + sorted((ROOT / "tools").glob("*.py")):
        try:
            ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        except Exception as exc:
            errors.append(f"{path.relative_to(ROOT)}: {exc}")

    try:
        flows = json.loads((ROOT / "node-red" / "flows.json").read_text(encoding="utf-8"))
        if not isinstance(flows, list) or not flows:
            errors.append("node-red/flows.json: expected a non-empty node list")
    except Exception as exc:
        errors.append(f"node-red/flows.json: {exc}")

    mission = yaml.safe_load((PACKAGE / "missions" / "factory_template.yaml").read_text(encoding="utf-8"))
    if mission.get("calibrated") is not False:
        errors.append("factory_template.yaml must remain locked until site calibration")

    if errors:
        print("Validation failed:")
        for error in errors:
            print(f"  - {error}")
        raise SystemExit(1)
    print("Validated ROS XML, YAML, Python syntax, Node-RED JSON, and mission safety lock.")


if __name__ == "__main__":
    validate()
