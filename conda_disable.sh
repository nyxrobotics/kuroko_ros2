#!/usr/bin/env bash
# Disable conda and force system Python for ROS 2 builds/runtime.
# This script is intended to be sourced, not executed.

# Deactivate conda environments if active
if command -v conda >/dev/null 2>&1; then
  while [[ -n "${CONDA_SHLVL:-}" && "${CONDA_SHLVL}" -gt 0 ]]; do
    conda deactivate || break
  done
fi

# Remove conda paths from PATH
if [[ -n "${PATH:-}" ]]; then
  export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v "/anaconda" | paste -sd ':' -)
fi

# Unset conda-related variables
unset CONDA_PREFIX
unset CONDA_DEFAULT_ENV
unset CONDA_PROMPT_MODIFIER
unset CONDA_SHLVL
unset PYTHONHOME

# Keep PYTHONPATH only for ROS (do not fully unset)
if [[ -n "${PYTHONPATH:-}" ]]; then
  export PYTHONPATH=$(echo "$PYTHONPATH" | tr ':' '\n' | grep -v "/anaconda" | paste -sd ':' -)
fi

# Reset command hash table
hash -r

# Diagnostics
echo "[conda_disable] python3 = $(which python3)"
python3 - << 'EOF'
import sys
print("[conda_disable] sys.executable =", sys.executable)
EOF

# Drop broken CMAKE_PREFIX_PATH entries (optional)
if [[ -n "${CMAKE_PREFIX_PATH:-}" ]]; then
  export CMAKE_PREFIX_PATH=$(echo "$CMAKE_PREFIX_PATH" | tr ';' '\n' | tr ':' '\n' | awk 'NF' | while read -r p; do
    [[ -d "$p" ]] && echo "$p"
  done | paste -sd ';' -)
fi