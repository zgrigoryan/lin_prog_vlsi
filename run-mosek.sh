#!/bin/sh
set -e

INPUT="${INPUT:-}"
BENCHMARK="${BENCHMARK:-apte}"
MCNC_DIR="${MCNC_DIR:-mcnc_hard}"
MODE="${MODE:-SA-CT-LP}"
ITERATIONS="${ITERATIONS:-1000}"
MAX_NO_IMPROVE_EPOCHS="${MAX_NO_IMPROVE_EPOCHS:-1000000}"
AUTO_TEMPERATURE="${AUTO_TEMPERATURE:-0}"
AUTO_EPOCH_LENGTH="${AUTO_EPOCH_LENGTH:-0}"
VERBOSE_SA="${VERBOSE_SA:-0}"
OUTPUT="${OUTPUT:-out/${BENCHMARK}_mosek_check}"
MOSEK_HOME="${MOSEK_HOME:-/Users/zara/Downloads/mosek}"
MOSEK_VERSION="${MOSEK_VERSION:-11.1}"
PYTHON="${PYTHON:-python3}"
BUILD_DIR="${BUILD_DIR:-build}"
SA_EXTRA_ARGS=""
if [ "${AUTO_TEMPERATURE}" = "1" ]; then
    SA_EXTRA_ARGS="${SA_EXTRA_ARGS} --auto-temperature"
fi
if [ "${AUTO_EPOCH_LENGTH}" = "1" ]; then
    SA_EXTRA_ARGS="${SA_EXTRA_ARGS} --auto-epoch-length"
fi
if [ "${VERBOSE_SA}" = "1" ]; then
    SA_EXTRA_ARGS="${SA_EXTRA_ARGS} --verbose-sa"
fi

if [ ! -x "${BUILD_DIR}/floorplanner" ]; then
    echo "floorplanner executable not found. Build first:"
    echo "  cmake -S . -B build -DFP_WITH_MOSEK=ON -DMOSEK_HOME=/Users/zara/Downloads/mosek -DMOSEK_VERSION=11.1"
    echo "  cmake --build build"
    echo "Or set BUILD_DIR=/path/to/build."
    exit 1
fi

MOSEK_EXE=$(find "${MOSEK_HOME}/${MOSEK_VERSION}/tools/platform" \
    -maxdepth 4 \
    -name mosek \
    -type f \
    -print 2>/dev/null \
    | head -n 1 || true)

mkdir -p "${OUTPUT}"

echo "Running floorplanner with native MOSEK backend"
set +e
if [ -n "${INPUT}" ]; then
    "${BUILD_DIR}/floorplanner" \
        --input "${INPUT}" \
        --mode "${MODE}" \
        --solver mosek \
        --iterations "${ITERATIONS}" \
        --max-no-improve-epochs "${MAX_NO_IMPROVE_EPOCHS}" \
        ${SA_EXTRA_ARGS} \
        --output "${OUTPUT}" \
        --export-mps "${OUTPUT}/model.mps" \
        --export-lp "${OUTPUT}/model.lp"
else
    "${BUILD_DIR}/floorplanner" \
        --mcnc "${BENCHMARK}" \
        --mcnc-dir "${MCNC_DIR}" \
        --mode "${MODE}" \
        --solver mosek \
        --iterations "${ITERATIONS}" \
        --max-no-improve-epochs "${MAX_NO_IMPROVE_EPOCHS}" \
        ${SA_EXTRA_ARGS} \
        --output "${OUTPUT}" \
        --export-mps "${OUTPUT}/model.mps" \
        --export-lp "${OUTPUT}/model.lp"
fi
floorplanner_status=$?
set -e

if [ -f "visualize_floorplan.py" ] && "${PYTHON}" -c "import matplotlib" >/dev/null 2>&1; then
    "${PYTHON}" visualize_floorplan.py "${OUTPUT}" --output "${OUTPUT}/floorplan.png"
else
    echo "Skipping plot: visualize_floorplan.py or matplotlib is unavailable."
fi

if [ "${floorplanner_status}" -ne 0 ]; then
    echo "floorplanner exited with status ${floorplanner_status}; continuing to solver check if an MPS was exported."
fi

echo
echo "Verifying exported MPS with MOSEK CLI"
if [ -z "${MOSEK_EXE}" ] || [ ! -x "${MOSEK_EXE}" ]; then
    echo "Skipping MOSEK CLI verification: executable not found under ${MOSEK_HOME}/${MOSEK_VERSION}/tools/platform"
elif [ -f "${OUTPUT}/model.mps" ]; then
    "${MOSEK_EXE}" "${OUTPUT}/model.mps"
else
    echo "No exported MPS found at ${OUTPUT}/model.mps"
fi

echo
echo "Summary:"
echo "  floorplanner summary: ${OUTPUT}/summary.json"
echo "  exported MPS:         ${OUTPUT}/model.mps"
echo "  exported LP:          ${OUTPUT}/model.lp"
echo "  MOSEK executable:     ${MOSEK_EXE}"
echo "  floorplan figure:     ${OUTPUT}/floorplan.png"
