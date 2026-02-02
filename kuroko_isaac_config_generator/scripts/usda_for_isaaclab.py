#!/usr/bin/env python3
from __future__ import annotations

import argparse
import fnmatch
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def _read_exclude_patterns(yaml_path: Path) -> list[str]:
    # Minimal YAML reader (no external deps). Expected shape:
    # exclude_from_articulation:
    #   - "pattern"
    if not yaml_path.exists():
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
            # strip quotes if present
            if (item.startswith('"') and item.endswith('"')) or (item.startswith("'") and item.endswith("'")):
                item = item[1:-1]
            if item:
                patterns.append(item)
        else:
            # stop if a new key appears
            if ":" in line:
                break
    return patterns


class SdfJointInfo:
    __slots__ = ("name", "type", "velocity_rad_s")
    def __init__(self, name: str, jtype: str, velocity_rad_s: float):
        self.name = name
        self.type = jtype
        self.velocity_rad_s = velocity_rad_s


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
    # Update articulation root apiSchemas: add PhysxArticulationAPI
    def repl_api(m: re.Match) -> str:
        pre = m.group(1)
        items = m.group(2)
        # Parse tokens inside [...]
        toks = re.findall(r'"([^"]+)"', items)
        if "PhysxArticulationAPI" not in toks:
            toks.append("PhysxArticulationAPI")
        # Keep PhysicsArticulationRootAPI first if present
        if "PhysicsArticulationRootAPI" in toks:
            toks = ["PhysicsArticulationRootAPI"] + [t for t in toks if t != "PhysicsArticulationRootAPI"]
        return f'{pre}["' + '", "'.join(toks) + '"]'
    usda = re.sub(
        r'(def\s+Xform\s+"Robot"\s*\(\s*\n\s*prepend\s+apiSchemas\s*=\s*)\[(.*?)\]',
        repl_api,
        usda,
        flags=re.DOTALL,
    )
    # Update relationship paths that reference /kuroko...
    usda = usda.replace("</kuroko", "</Robot")
    usda = usda.replace('="/kuroko', '="/Robot')
    return usda


def _process_joint_block(block: str, joint_name: str, sdf_joints: dict[str, SdfJointInfo], exclude_patterns: list[str]) -> str:
    axis = "angular"
    info = sdf_joints.get(joint_name)
    if info and info.type.lower().startswith("pris"):
        axis = "linear"
    elif info and info.type.lower().startswith("rev"):
        axis = "angular"

    # ① excludeFromArticulation
    if any(fnmatch.fnmatch(joint_name, pat) for pat in exclude_patterns):
        if re.search(r'\bphysics:excludeFromArticulation\b', block):
            block = re.sub(r'\bbool\s+physics:excludeFromArticulation\s*=\s*\d+\s*', 'bool physics:excludeFromArticulation = 1', block)
        else:
            # insert right after opening '{'
            block = re.sub(r'\{\n', '{\n        bool physics:excludeFromArticulation = 1\n', block, count=1)

    def _find_max_force(text: str, drive_axis: str) -> float | None:
        m2 = re.search(rf'\bphysics:drive:{re.escape(drive_axis)}:maxForce\s*=\s*([0-9eE+\-.]+)', text)
        if not m2:
            return None
        try:
            return float(m2.group(1))
        except ValueError:
            return None

    def _remove_drive_axis(text: str, drive_axis: str) -> str:
        # Remove drive attributes for this axis (tolerate type prefixes like "uniform float", "float", etc.)
        text = re.sub(
            rf'^\s*(?:\w+\s+)*physics:drive:{re.escape(drive_axis)}:[^\n]*\n',
            '',
            text,
            flags=re.MULTILINE,
        )
        # Remove PhysX drive envelope attributes for this axis as well
        text = re.sub(
            rf'^\s*(?:\w+\s+)*physxDrivePerformanceEnvelope:{re.escape(drive_axis)}:[^\n]*\n',
            '',
            text,
            flags=re.MULTILINE,
        )

        # Remove applied API token(s)
        text = re.sub(rf'"PhysicsDriveAPI:{re.escape(drive_axis)}"\s*,?\s*', '', text)
        text = re.sub(rf'"PhysxDrivePerformanceEnvelopeAPI:{re.escape(drive_axis)}"\s*,?\s*', '', text)

        # Clean up apiSchemas list formatting
        text = re.sub(r'apiSchemas\s*=\s*\[\s*,', 'apiSchemas = [', text)
        text = re.sub(r',\s*\]', ']', text)
        text = re.sub(r'\[\s*\]', '[]', text)
        return text

    # ③/④ detect drive maxForce per axis and gate behavior
    max_force_axis = _find_max_force(block, axis)

    # Remove drive for any axis that exists and is effectively disabled
    for drive_axis in ("angular", "linear"):
        mf = _find_max_force(block, drive_axis)
        if mf is not None and mf <= 1e-6:
            block = _remove_drive_axis(block, drive_axis)

    # ④ apply velocity from SDF only for actuated joints (Max Force > 1e-6)
    if max_force_axis is not None and max_force_axis > 1e-6 and info and info.velocity_rad_s and info.velocity_rad_s > 0:
        vel_deg = info.velocity_rad_s * 180.0 / math.pi
        vel_deg_str = f"{vel_deg:.6f}".rstrip("0").rstrip(".")
        # Ensure PhysxJointAxisAPI applied and set maxJointVelocity (Maximum Joint Velocity)
        if "PhysxJointAxisAPI:%s" % axis not in block:
            block = re.sub(r'(prepend\s+apiSchemas\s*=\s*\[)([^\]]*)\]', lambda m2: m2.group(1) + m2.group(2).rstrip() + (", " if m2.group(2).strip() else "") + f'"PhysxJointAxisAPI:{axis}"]', block, flags=re.DOTALL, count=1)
        if re.search(rf'physxJointAxis:{re.escape(axis)}:maxJointVelocity\b', block):
            block = re.sub(rf'(physxJointAxis:{re.escape(axis)}:maxJointVelocity\s*=\s*)([0-9eE+\-.]+)', rf'\g<1>{vel_deg_str}', block)
        else:
            block = re.sub(r'\{\n', '{\n        float physxJointAxis:%s:maxJointVelocity = %s\n' % (axis, vel_deg_str), block, count=1)

        # Ensure PhysxDrivePerformanceEnvelopeAPI applied and set maxActuatorVelocity (Max Actuator Velocity)
        if "PhysxDrivePerformanceEnvelopeAPI:%s" % axis not in block:
            block = re.sub(r'(prepend\s+apiSchemas\s*=\s*\[)([^\]]*)\]', lambda m2: m2.group(1) + m2.group(2).rstrip() + (", " if m2.group(2).strip() else "") + f'"PhysxDrivePerformanceEnvelopeAPI:{axis}"]', block, flags=re.DOTALL, count=1)
        if re.search(rf'physxDrivePerformanceEnvelope:{re.escape(axis)}:maxActuatorVelocity\b', block):
            block = re.sub(rf'(physxDrivePerformanceEnvelope:{re.escape(axis)}:maxActuatorVelocity\s*=\s*)([0-9eE+\-.]+)', rf'\g<1>{vel_deg_str}', block)
        else:
            block = re.sub(r'\{\n', '{\n        float physxDrivePerformanceEnvelope:%s:maxActuatorVelocity = %s\n' % (axis, vel_deg_str), block, count=1)

    return block


def _edit_usda(input_usda: Path, input_sdf: Path, output_usda: Path, exclude_yaml: Path) -> None:
    usda = input_usda.read_text(encoding="utf-8")

    # ⑤ stage metadata + rename root
    usda = _ensure_header_block(usda)
    usda = _replace_root_name(usda)

    sdf_joints = _parse_sdf_joints(input_sdf)
    exclude_patterns = _read_exclude_patterns(exclude_yaml)

    # Process each joint block (best-effort brace matching at the joint prim level)
    out_lines: list[str] = []
    lines = usda.splitlines(True)

    joint_start_re = re.compile(r'^(\s*def\s+\w*Joint\s+"([^"]+)"\s*\()', re.MULTILINE)
    i = 0
    while i < len(lines):
        line = lines[i]
        m = joint_start_re.match(line)
        if not m:
            out_lines.append(line)
            i += 1
            continue

        joint_name = m.group(2)
        # Collect until matching braces for this prim
        block = [line]
        i += 1

        # Copy lines until we see first '{'
        brace_depth = 0
        seen_open = False
        while i < len(lines):
            block.append(lines[i])
            if "{" in lines[i]:
                brace_depth += lines[i].count("{")
                seen_open = True
            if "}" in lines[i]:
                brace_depth -= lines[i].count("}")
            i += 1
            if seen_open and brace_depth <= 0:
                break

        block_text = "".join(block)
        block_text = _process_joint_block(block_text, joint_name, sdf_joints, exclude_patterns)
        out_lines.append(block_text)

    output_usda.write_text("".join(out_lines), encoding="utf-8")


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
