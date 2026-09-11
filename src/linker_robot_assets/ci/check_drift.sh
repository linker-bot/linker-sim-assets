#!/usr/bin/env bash
# CI drift check: re-run the composer on every workstation and unit and verify
# that the committed workstation.urdf / workstation.xml / manifest.yaml match a
# fresh compose. Any mismatch means someone edited a recipe or component
# source without re-running the composer.
#
# Usage (run from anywhere — paths resolve relative to this script):
#   bash src/linker_robot_assets/ci/check_drift.sh              # check all workstations + units
#   bash src/linker_robot_assets/ci/check_drift.sh <name>       # check one workstation or unit
#
# Optionally also enforce semver on joint-name renames vs the previous
# git tag (refuses minor/patch bumps when joint names rename):
#   CHECK_JOINT_RENAMES=1 bash …/check_drift.sh
#
# Exit code: 0 if clean, 1 if drift / version violation, 2 on usage errors.
#
# Requires: linker-robot-assets[authoring] installed in the active env, OR
# src on PYTHONPATH (a fresh checkout sets this via pyproject.toml's
# [tool.pytest.ini_options].pythonpath; for direct invocation outside
# pytest, prefix with PYTHONPATH=… ).

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Layout: HERE = <repo-root>/src/linker_robot_assets/ci/
#         REPO_ROOT = HERE/../../.. (climb 3 levels)
REPO_ROOT="$(cd "${HERE}/../../.." && pwd)"
WORKSTATIONS_DIR="${HERE}/../assets/workstations"
UNITS_DIR="${HERE}/../assets/units"
PYTHON="${PYTHON:-python3}"

if [[ $# -gt 1 ]]; then
    echo "usage: $0 [workstation_or_unit_name]" >&2
    exit 2
fi

if [[ $# -eq 1 ]]; then
    # Single target: look in workstations/ first, then units/.
    if [[ -d "${WORKSTATIONS_DIR}/$1" ]]; then
        TARGETS=("${WORKSTATIONS_DIR}/$1")
    else
        TARGETS=("${UNITS_DIR}/$1")
    fi
else
    shopt -s nullglob
    TARGETS=()
    for base in "${WORKSTATIONS_DIR}" "${UNITS_DIR}"; do
        for d in "${base}"/*/; do
            if [[ -f "${d}/recipe.yaml" ]]; then
                TARGETS+=("${d%/}")
            fi
        done
    done
fi

if [[ ${#TARGETS[@]} -eq 0 ]]; then
    echo "no workstations or units found under ${WORKSTATIONS_DIR} / ${UNITS_DIR}" >&2
    exit 2
fi

cd "${REPO_ROOT}"

fail=0
for ws in "${TARGETS[@]}"; do
    name="$(basename "${ws}")"
    echo "[check_drift] ${name}"
    if ! "${PYTHON}" -m linker_robot_assets.composer.compose "${ws}" --check-drift; then
        fail=1
    fi
done

# Solo units under units/ carry a *generated* recipe.yaml; verify it still
# matches the deriver (catches a component or generator change that wasn't
# re-committed — the per-dir compose above only checks artifacts vs recipe).
echo "[check_drift] derive_solo (generated solo recipes)"
if ! "${PYTHON}" -m linker_robot_assets.composer.derive_solo --check-drift; then
    fail=1
fi

if [[ ${fail} -ne 0 ]]; then
    echo >&2
    echo "drift detected. Re-run:" >&2
    echo "  for ws in src/linker_robot_assets/assets/workstations/*/ src/linker_robot_assets/assets/units/*/; do python -m linker_robot_assets.composer.compose \"\$ws\"; done" >&2
    echo "  python -m linker_robot_assets.composer.derive_solo" >&2
    echo "and commit the updated artifacts." >&2
    exit 1
fi
echo "all workstations and units clean."

# Optional semver / joint-rename guard against the previous git tag.
# Off by default so locally invoking the drift check stays cheap.
if [[ "${CHECK_JOINT_RENAMES:-0}" == "1" ]]; then
    echo "[check_drift] checking joint-name stability vs previous tag…"
    if ! "${PYTHON}" -m linker_robot_assets.ci.joint_rename_guard; then
        exit 1
    fi
fi
