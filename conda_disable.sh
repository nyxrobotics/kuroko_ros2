#!/usr/bin/env bash
# Usage:
#   source /path/to/conda_disable_source.sh
#
# Removes Anaconda/Conda paths from the CURRENT shell environment (must be sourced).
# - Removes any PATH entry that is exactly:
#     /home/<user>/anaconda3/bin
#     /home/<user>/anaconda3/condabin
# - Also removes any other entries that contain "/anaconda3/" just in case.

set -euo pipefail

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "[conda_disable] ERROR: This script must be sourced, not executed."
  echo "[conda_disable]   Use: source ${BASH_SOURCE[0]}"
  return 2 2>/dev/null || exit 2
fi

# Drop PATH entries that match the patterns (handles duplicates).
_clean_path() {
  awk -v RS=: -v ORS=: '
    $0 == "" { next }
    $0 ~ /\/anaconda3\/bin$/ { next }
    $0 ~ /\/anaconda3\/condabin$/ { next }
    $0 ~ /\/anaconda3\// { next }
    { print $0 }
  ' <<< "${PATH:-}" | sed 's/:$//'
}

export PATH="$(_clean_path)"

# Defensive: if conda is active, deactivate it (won't error if not available).
if type conda >/dev/null 2>&1; then
  # shellcheck disable=SC1091
  conda deactivate >/dev/null 2>&1 || true
fi

hash -r

echo "[conda_disable] PATH = ${PATH}"
echo "[conda_disable] python3 = $(command -v python3 || true)"
python3 - <<'PY'
import sys
print("[conda_disable] sys.executable =", sys.executable)
PY