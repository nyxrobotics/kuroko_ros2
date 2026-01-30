import os
import re
from pathlib import Path
from ament_index_python.packages import get_package_share_directory


_PACKAGE_URI_RE = re.compile(r"^package://([^/]+)/(.+)$")


def resolve_package_uri(uri: str) -> str | None:
    m = _PACKAGE_URI_RE.match(uri.strip())
    if not m:
        return None
    pkg, rel = m.group(1), m.group(2)
    try:
        share = get_package_share_directory(pkg)
    except Exception:
        return None
    return str(Path(share) / rel)


def resolve_model_uri(uri: str) -> str | None:
    # model://<name>/<path> using GAZEBO_MODEL_PATH
    if not uri.startswith("model://"):
        return None
    rest = uri[len("model://") :]
    parts = rest.split("/", 1)
    model_name = parts[0]
    tail = parts[1] if len(parts) > 1 else ""
    search_paths = os.environ.get("GAZEBO_MODEL_PATH", "").split(os.pathsep)
    for p in [Path(x) for x in search_paths if x]:
        candidate = p / model_name / tail
        if candidate.exists():
            return str(candidate)
    return None


def resolve_any_uri(uri: str) -> str | None:
    uri = uri.strip()
    if uri.startswith("package://"):
        return resolve_package_uri(uri)
    if uri.startswith("model://"):
        return resolve_model_uri(uri)
    if uri.startswith("file://"):
        return uri[len("file://") :]
    # absolute or relative path
    return None
