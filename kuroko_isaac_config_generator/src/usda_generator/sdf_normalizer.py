import re
from pathlib import Path

from ros_package_path import resolve_any_uri


_URI_TAG_RE = re.compile(r"(<uri>)(.*?)(</uri>)", re.DOTALL)


def rewrite_sdf_uris_to_absolute(sdf_xml: str, base_dir: str | None = None) -> str:
    base = Path(base_dir) if base_dir else None

    def repl(match):
        prefix, uri, suffix = match.group(1), match.group(2).strip(), match.group(3)
        resolved = resolve_any_uri(uri)
        if resolved:
            return f"{prefix}{resolved}{suffix}"
        if base and not (uri.startswith("/") or "://" in uri):
            cand = (base / uri).resolve()
            return f"{prefix}{str(cand)}{suffix}"
        return match.group(0)

    return _URI_TAG_RE.sub(repl, sdf_xml)
