#!/usr/bin/env python3
from __future__ import annotations

import argparse
import fnmatch
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


class SdfJointInfo:
    __slots__ = ("name", "type", "velocity_rad_s")

    def __init__(self, name: str, jtype: str, velocity_rad_s: float):
        self.name = name
        self.type = jtype
        self.velocity_rad_s = velocity_rad_s



def _read_exclude_patterns(yaml_path: Path) -> list[str]:
    # Minimal YAML reader (no external deps). Expected shape:
    # exclude_from_articulation:
    #   - "pattern"
    #
    # If the file does not exist, create a template file and return an empty list.
    if not yaml_path.exists():
        yaml_path.parent.mkdir(parents=True, exist_ok=True)
        yaml_path.write_text(
            "exclude_from_articulation:\n"
            "  # Examples:\n"
            "  # - \"*_loop\"\n"
            "  # - \"head_joint\"\n",
            encoding="utf-8",
        )
        print(f"[INFO] Created exclude yaml template: {yaml_path}")
        return []

    patterns: list[str] = []
    in_list = False
    for raw in yaml_path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("exclude_from_articulation:"):
            in_list = True
            continue
        if not in_list:
            continue
        if line.startswith("-"):
            item = line[1:].strip()
            if (item.startswith('"') and item.endswith('"')) or (item.startswith("'") and item.endswith("'")):
                item = item[1:-1]
            if item:
                patterns.append(item)
        else:
            # Stop if a new YAML key appears.
            if ":" in line:
                break

    print(f"[INFO] Loaded exclude patterns: {len(patterns)}")
    if patterns:
        for pat in patterns:
            print(f"  - {pat}")
    return patterns

def _parse_sdf_joints(sdf_path: Path) -> dict[str, SdfJointInfo]:
    # Works for model-only SDF like:
    # <sdf><model><joint name="..." type="revolute"><axis><limit><velocity>...</velocity>
    tree = ET.parse(str(sdf_path))
    root = tree.getroot()

    joints: dict[str, SdfJointInfo] = {}
    for joint in root.findall(".//joint"):
        name = joint.attrib.get("name", "")
        jtype = joint.attrib.get("type", "")
        vel = 0.0
        # SDF 1.7: <axis><limit><velocity>
        vel_el = joint.find("./axis/limit/velocity")
        if vel_el is None:
            vel_el = joint.find("./axis1/limit/velocity")  # some variants
        if vel_el is not None and (vel_el.text or "").strip():
            try:
                vel = float(vel_el.text.strip())
            except ValueError:
                vel = 0.0
        if name:
            joints[name] = SdfJointInfo(name, jtype, vel)
    return joints


def _ensure_header_block(usda: str) -> str:
    # Ensure the file starts with:
    # #usda 1.0
    # (
    #     defaultPrim = "Robot"
    #     metersPerUnit = 1
    #     upAxis = "Z"
    # )
    lines = usda.splitlines(True)
    if not lines:
        return usda
    if not lines[0].startswith("#usda"):
        return usda

    wanted = [
        "#usda 1.0\n",
        "\n",
        "(\n",
        '    defaultPrim = "Robot"\n',
        "    metersPerUnit = 1\n",
        '    upAxis = "Z"\n',
        ")\n",
        "\n",
    ]

    # If second non-empty line starts with "(", assume already present.
    non_empty = [i for i, l in enumerate(lines) if l.strip()]
    if len(non_empty) >= 2 and lines[non_empty[1]].lstrip().startswith("("):
        # Replace existing header block (best-effort) up to its closing ')'
        start = non_empty[1]
        end = start
        depth = 0
        for i in range(start, len(lines)):
            if "(" in lines[i]:
                depth += lines[i].count("(")
            if ")" in lines[i]:
                depth -= lines[i].count(")")
                if depth <= 0:
                    end = i
                    break
        new_lines = lines[:start] + wanted[2:] + lines[end+1:]
        # ensure first line is #usda 1.0
        new_lines[0] = "#usda 1.0\n"
        return "".join(new_lines)

    # Insert after first line
    return lines[0] + "".join(wanted[1:]) + "".join(lines[1:])


def _replace_root_name(usda: str) -> str:
    # Rename root prim "kuroko" -> "Robot"
    usda = re.sub(r'^(\s*def\s+Xform\s+)"kuroko"(\s*\()', r'\1"Robot"\2', usda, flags=re.MULTILINE)

    # Ensure the root prim has exactly the requested articulation schemas.
    # We intentionally overwrite the list to match the desired spec:
    # prepend apiSchemas = ["PhysicsArticulationRootAPI", "PhysxArticulationAPI"]
    def _force_root_api_schemas(m: re.Match) -> str:
        indent = m.group(1)
        return (
            f'{indent}prepend apiSchemas = ["PhysicsArticulationRootAPI", "PhysxArticulationAPI"]'
        )

    usda = re.sub(
        r'^(\s*)prepend\s+apiSchemas\s*=\s*\[[^\]]*\]\s*$',
        _force_root_api_schemas,
        usda,
        flags=re.MULTILINE,
        count=1,
    )

    # Update relationship paths that reference /kuroko...
    usda = usda.replace("</kuroko", "</Robot")
    usda = usda.replace('="/kuroko', '="/Robot')

    return usda


def _ensure_physics_scene_block(usda: str) -> str:
    # Insert a PhysicsScene prim under the Robot Xform if it does not exist.
    if re.search(r'^\s*def\s+PhysicsScene\s+\"PhysicsScene\"\b', usda, flags=re.MULTILINE):
        return usda

    m = re.search(r'^\s*def\s+Xform\s+\"Robot\"\b[^{]*\{', usda, flags=re.MULTILINE | re.DOTALL)
    if not m:
        print("[WARN] Robot Xform block not found; cannot insert PhysicsScene", file=sys.stderr)
        return usda

    line_start = usda.rfind("\n", 0, m.start()) + 1
    indent = re.match(r"(\s*)", usda[line_start:m.start()]).group(1)
    inner_indent = indent + "    "

    scene = (
        "\n"
        f"{inner_indent}def PhysicsScene \"PhysicsScene\" (\n"
        f"{inner_indent}    prepend apiSchemas = [\"PhysxSceneAPI\"]\n"
        f"{inner_indent})\n"
        f"{inner_indent}{{\n"
        f"{inner_indent}    uniform token physxScene:broadphaseType = \"MBP\"\n"
        f"{inner_indent}    bool physxScene:enableGPUDynamics = 0\n"
        f"{inner_indent}    uniform uint physxScene:maxPositionIterationCount = 16\n"
        f"{inner_indent}    uniform uint physxScene:maxVelocityIterationCount = 1\n"
        f"{inner_indent}    uniform uint physxScene:minPositionIterationCount = 16\n"
        f"{inner_indent}    uniform uint physxScene:minVelocityIterationCount = 1\n"
        f"{inner_indent}    uniform token physxScene:solverType = \"PGS\"\n"
        f"{inner_indent}    uint physxScene:timeStepsPerSecond = 500\n"
        f"{inner_indent}}}\n"
    )

    usda = usda[: m.end()] + scene + usda[m.end() :]
    print("[INFO] Inserted PhysicsScene under Robot")
    return usda

def _process_joint_block(
    block: str,
    joint_name: str,
    sdf_joints: dict[str, SdfJointInfo],
    exclude_patterns: list[str],
) -> str:
    info = sdf_joints.get(joint_name)

    # Determine the axis to use for velocity propagation.
    # - revolute / continuous: angular
    # - prismatic: linear
    primary_axis = "angular"
    if info and info.type.lower().startswith("pris"):
        primary_axis = "linear"

    # ① excludeFromArticulation (wildcards supported via fnmatch)
    if exclude_patterns and any(fnmatch.fnmatch(joint_name, pat) for pat in exclude_patterns):
        if re.search(r"\bphysics:excludeFromArticulation\b", block):
            block = re.sub(
                r"\bbool\s+physics:excludeFromArticulation\s*=\s*\d+\s*",
                "bool physics:excludeFromArticulation = 1",
                block,
            )
        else:
            block = re.sub(r"\{\n", "{\n        bool physics:excludeFromArticulation = 1\n", block, count=1)
        print(f"[INFO] excludeFromArticulation=1: {joint_name}")

    def _get_drive_max_force(text: str, axis: str) -> float | None:
        m = re.search(rf"\bphysics:drive:{re.escape(axis)}:maxForce\s*=\s*([0-9eE+\-.]+)", text)
        if not m:
            return None
        try:
            return float(m.group(1))
        except ValueError:
            return None

    def _strip_axis_drive(text: str, axis: str) -> str:
        # Remove any authored attributes for this axis.
        # Be tolerant of any type prefixes like "uniform float", "custom rel", etc.
        text = re.sub(
            rf"^\s*(?:\w+\s+)*\bphysics:drive:{re.escape(axis)}:[^\n]*\n",
            "",
            text,
            flags=re.MULTILINE,
        )
        text = re.sub(
            rf"^\s*(?:\w+\s+)*\bphysxDrivePerformanceEnvelope:{re.escape(axis)}:[^\n]*\n",
            "",
            text,
            flags=re.MULTILINE,
        )

        # Remove applied API tokens for this axis inside apiSchemas
        text = re.sub(rf'"PhysicsDriveAPI:{re.escape(axis)}"\s*,?\s*', "", text, flags=re.DOTALL)
        text = re.sub(rf'"PhysxDrivePerformanceEnvelopeAPI:{re.escape(axis)}"\s*,?\s*', "", text, flags=re.DOTALL)

        # Clean up any leftover commas or empty lists like [ , "X" ].
        text = re.sub(r"apiSchemas\s*=\s*\[\s*,", "apiSchemas = [", text)
        text = re.sub(r",\s*,+", ", ", text)
        text = re.sub(r"\[\s*\]", "[]", text)
        text = re.sub(r",\s*\]", "]", text)
        return text

    def _deg_per_sec_from_rad(rad_s: float) -> str:
        deg = rad_s * 180.0 / math.pi
        return f"{deg:.6f}".rstrip("0").rstrip(".")

    def _set_or_add_attr(text: str, attr: str, value: str) -> str:
        # Replace existing
        if re.search(rf"\b{re.escape(attr)}\b", text):
            return re.sub(
                rf"({re.escape(attr)}\s*=\s*)([0-9eE+\-.]+)",
                rf"\g<1>{value}",
                text,
            )
        # Add right after "{\n"
        return re.sub(r"\{\n", "{\n        float %s = %s\n" % (attr, value), text, count=1)

    def _ensure_api(text: str, api_token: str) -> str:
        if api_token in text:
            return text
        # Append to the first apiSchemas list in this block.
        def _append(m: re.Match) -> str:
            head = m.group(1)
            body = m.group(2)
            body_stripped = body.strip()
            if not body_stripped:
                return f'{head}"{api_token}"]'
            # ensure trailing comma
            body2 = body.rstrip()
            if not body2.rstrip().endswith(","):
                body2 = body2.rstrip() + ", "
            return f'{head}{body2}"{api_token}"]'

        return re.sub(r"(prepend\s+apiSchemas\s*=\s*\[)([^\]]*)\]", _append, text, flags=re.DOTALL, count=1)

    # --- Drive removal / velocity propagation ---
    # ③ If maxForce <= 1e-6 for an axis, remove Drive components for that axis.
    # ④ If maxForce > 1e-6 for an axis, propagate SDF velocity to:
    #    - Maximum Joint Velocity: physxJointAxis:<axis>:maxJointVelocity
    #    - Drive->Advanced->Max Actuator Velocity: physxDrivePerformanceEnvelope:<axis>:maxActuatorVelocity
    #
    # IMPORTANT: Only do (④) for joints/axes where maxForce > 1e-6.
    for axis in ("angular", "linear"):
        max_force = _get_drive_max_force(block, axis)
        if max_force is None:
            continue

        if max_force <= 1e-6:
            print(f"[INFO] Remove drive (maxForce={max_force:g}) joint={joint_name} axis={axis}")
            block = _strip_axis_drive(block, axis)
            continue

        # Actuated axis: only apply velocities to the joint's primary axis.
        if axis != primary_axis:
            print(f"[INFO] Skip velocity (non-primary axis) joint={joint_name} axis={axis} maxForce={max_force:g}")
            continue

        if not info:
            print(f"[WARN] No SDF joint info for actuated joint: {joint_name} (axis={axis})")
            continue

        if not info.velocity_rad_s or info.velocity_rad_s <= 0:
            print(f"[WARN] SDF velocity missing/<=0 for joint={joint_name}: {info.velocity_rad_s}")
            continue

        vel_deg_str = _deg_per_sec_from_rad(info.velocity_rad_s)
        print(
            f"[INFO] Apply velocity joint={joint_name} axis={axis} "
            f"maxForce={max_force:g} sdf={info.velocity_rad_s:g} rad/s -> {vel_deg_str} deg/s"
        )

        # Ensure APIs exist
        block = _ensure_api(block, f"PhysxJointAxisAPI:{axis}")
        block = _ensure_api(block, f"PhysxDrivePerformanceEnvelopeAPI:{axis}")

        # Set attributes
        block = _set_or_add_attr(block, f"physxJointAxis:{axis}:maxJointVelocity", vel_deg_str)
        block = _set_or_add_attr(block, f"physxDrivePerformanceEnvelope:{axis}:maxActuatorVelocity", vel_deg_str)

    return block

def _edit_usda(input_usda: Path, input_sdf: Path, output_usda: Path, exclude_yaml: Path) -> None:
    usda = input_usda.read_text(encoding="utf-8")

    # ⑤ stage metadata + rename root
    usda = _ensure_header_block(usda)
    usda = _replace_root_name(usda)
    usda = _ensure_physics_scene_block(usda)

    sdf_joints = _parse_sdf_joints(input_sdf)
    print(f"[INFO] Parsed SDF joints: {len(sdf_joints)}")
    exclude_patterns = _read_exclude_patterns(exclude_yaml)

    # Process each joint block (best-effort brace matching at the joint prim level)
    out_lines: list[str] = []
    lines = usda.splitlines(True)

    joint_start_re = re.compile(r'^(\s*def\s+\w*Joint\s+"([^"]+)"\s*\()', re.MULTILINE)
    i = 0
    processed = 0
    while i < len(lines):
        line = lines[i]
        m = joint_start_re.match(line)
        if not m:
            out_lines.append(line)
            i += 1
            continue

        joint_name = m.group(2)
        processed += 1

        # Collect until matching braces for this prim
        block_lines = [line]
        i += 1

        brace_depth = 0
        seen_open = False
        while i < len(lines):
            block_lines.append(lines[i])
            if "{" in lines[i]:
                brace_depth += lines[i].count("{")
                seen_open = True
            if "}" in lines[i]:
                brace_depth -= lines[i].count("}")
            i += 1
            if seen_open and brace_depth <= 0:
                break

        block_text = "".join(block_lines)
        block_text = _process_joint_block(block_text, joint_name, sdf_joints, exclude_patterns)
        out_lines.append(block_text)

    print(f"[INFO] Processed joint prims: {processed}")
    output_usda.write_text("".join(out_lines), encoding="utf-8")
    print(f"[INFO] Wrote output: {output_usda}")

def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--input_usda", type=Path, required=True)
    ap.add_argument("--input_sdf", type=Path, required=True)
    ap.add_argument("--output_usda", type=Path, required=True)
    ap.add_argument("--exclude_yaml", type=Path, required=True)
    args = ap.parse_args()

    if not args.input_usda.exists():
        raise FileNotFoundError(args.input_usda)
    if not args.input_sdf.exists():
        raise FileNotFoundError(args.input_sdf)
    args.output_usda.parent.mkdir(parents=True, exist_ok=True)

    _edit_usda(args.input_usda, args.input_sdf, args.output_usda, args.exclude_yaml)


if __name__ == "__main__":
    main()
