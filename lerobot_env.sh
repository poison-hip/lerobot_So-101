#!/usr/bin/env bash
set -euo pipefail

# Usage examples:
#   source ./lerobot_env.sh
#   ./lerobot_env.sh -- lerobot-info
#   ./lerobot_env.sh -- python -m lerobot_side.server

ENV_PATH="${LEROBOT_CONDA_ENV:-$HOME/gpufree-data/tmp/conda_envs/env_lerobot}"
LEROBOT_ROOT="${LEROBOT_ROOT:-$HOME/gpufree-data/lerobot}"

_source_conda_sh() {
  if [ -f "/opt/conda/etc/profile.d/conda.sh" ]; then
    # shellcheck disable=SC1091
    source "/opt/conda/etc/profile.d/conda.sh"
    return
  fi
  if [ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]; then
    # shellcheck disable=SC1091
    source "$HOME/miniconda3/etc/profile.d/conda.sh"
    return
  fi
  echo "[ERROR] conda.sh not found. Please install/initialize conda first." >&2
  exit 1
}

_source_conda_sh
conda activate "$ENV_PATH"

if [ ! -d "$LEROBOT_ROOT" ]; then
  echo "[ERROR] LEROBOT_ROOT not found: $LEROBOT_ROOT" >&2
  exit 1
fi
cd "$LEROBOT_ROOT"

echo "[INFO] env_lerobot activated: $CONDA_PREFIX"
echo "[INFO] LEROBOT_ROOT: $LEROBOT_ROOT"
echo "[INFO] Python: $(command -v python)"
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "[INFO] Running as a subprocess. Use 'source ./lerobot_env.sh' to keep env in current shell."
fi

if [ "${1:-}" = "--" ]; then
  shift
fi

if [ "$#" -gt 0 ]; then
  exec "$@"
fi

exec lerobot-info
