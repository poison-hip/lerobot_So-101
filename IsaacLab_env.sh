#!/usr/bin/env bash
set -euo pipefail

# Usage examples:
#   source ./IsaacLab_env.sh
#   ./IsaacLab_env.sh -- python -c "import isaaclab; print('ok')"
#   ./IsaacLab_env.sh -- ./isaaclab.sh -p scripts/tutorials/00_sim/create_empty.py

ENV_PATH="${ISAACLAB_CONDA_ENV:-$HOME/gpufree-data/tmp/conda_envs/env_isaaclab}"
ISAACLAB_ROOT="${ISAACLAB_ROOT:-$HOME/gpufree-data/tmp/projects/IsaacLab}"

_source_conda_sh() {
  if [ -f "/opt/conda/etc/profile.d/conda.sh" ]; then
    # Typical container install.
    # shellcheck disable=SC1091
    source "/opt/conda/etc/profile.d/conda.sh"
    return
  fi
  if [ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]; then
    # Typical local miniconda install.
    # shellcheck disable=SC1091
    source "$HOME/miniconda3/etc/profile.d/conda.sh"
    return
  fi
  echo "[ERROR] conda.sh not found. Please install/initialize conda first." >&2
  exit 1
}

_activate_env() {
  _source_conda_sh
  conda activate "$ENV_PATH"
}

_setup_isaaclab_env() {
  if [ ! -d "$ISAACLAB_ROOT" ]; then
    echo "[ERROR] ISAACLAB_ROOT not found: $ISAACLAB_ROOT" >&2
    exit 1
  fi

  cd "$ISAACLAB_ROOT"
  if [ ! -f "_isaac_sim/setup_conda_env.sh" ]; then
    echo "[ERROR] Missing _isaac_sim/setup_conda_env.sh in $ISAACLAB_ROOT" >&2
    exit 1
  fi

  # IsaacLab official script may reference unset shell vars (e.g., ZSH_VERSION).
  # Temporarily disable nounset to avoid aborting here.
  set +u
  # shellcheck disable=SC1091
  source "_isaac_sim/setup_conda_env.sh"
  set -u

  export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:${LD_LIBRARY_PATH:-}"
  export PYTHONPATH="$ISAACLAB_ROOT/source:${PYTHONPATH:-}"
}

_activate_env
_setup_isaaclab_env

echo "[INFO] env_isaaclab activated: $CONDA_PREFIX"
echo "[INFO] ISAACLAB_ROOT: $ISAACLAB_ROOT"
echo "[INFO] Python: $(command -v python)"
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "[INFO] Running as a subprocess. Use 'source ./IsaacLab_env.sh' to keep env in current shell."
fi

if [ "${1:-}" = "--" ]; then
  shift
fi

if [ "$#" -gt 0 ]; then
  exec "$@"
fi

echo "[INFO] Ready. Example:"
echo "       ./IsaacLab_env.sh -- ./isaaclab.sh -p scripts/tutorials/00_sim/create_empty.py"
