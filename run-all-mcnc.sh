#!/bin/sh
set -e

BENCHMARKS="${BENCHMARKS:-apte xerox hp ami33 ami49}"
ITERATIONS="${ITERATIONS:-1000}"
MODE="${MODE:-SA-CT-LP}"
ROOT="${ROOT:-out/mcnc_${MODE}}"
RUN_HIGHS="${RUN_HIGHS:-1}"
RUN_MOSEK="${RUN_MOSEK:-1}"
AUTO_TEMPERATURE="${AUTO_TEMPERATURE:-0}"
AUTO_EPOCH_LENGTH="${AUTO_EPOCH_LENGTH:-0}"
VERBOSE_SA="${VERBOSE_SA:-0}"
BUILD_DIR="${BUILD_DIR:-build}"
HIGHS_BUILD_DIR="${HIGHS_BUILD_DIR:-${BUILD_DIR}}"
MOSEK_BUILD_DIR="${MOSEK_BUILD_DIR:-${BUILD_DIR}}"

for benchmark in ${BENCHMARKS}; do
    if [ "${RUN_HIGHS}" = "1" ]; then
        echo
        echo "=== ${benchmark}: HiGHS (${MODE}, ${ITERATIONS} iterations) ==="
        BENCHMARK="${benchmark}" \
        MODE="${MODE}" \
        ITERATIONS="${ITERATIONS}" \
        AUTO_TEMPERATURE="${AUTO_TEMPERATURE}" \
        AUTO_EPOCH_LENGTH="${AUTO_EPOCH_LENGTH}" \
        VERBOSE_SA="${VERBOSE_SA}" \
        BUILD_DIR="${HIGHS_BUILD_DIR}" \
        OUTPUT="${ROOT}/${benchmark}_highs" \
        sh run-highs.sh || true
    fi

    if [ "${RUN_MOSEK}" = "1" ]; then
        echo
        echo "=== ${benchmark}: MOSEK (${MODE}, ${ITERATIONS} iterations) ==="
        BENCHMARK="${benchmark}" \
        MODE="${MODE}" \
        ITERATIONS="${ITERATIONS}" \
        AUTO_TEMPERATURE="${AUTO_TEMPERATURE}" \
        AUTO_EPOCH_LENGTH="${AUTO_EPOCH_LENGTH}" \
        VERBOSE_SA="${VERBOSE_SA}" \
        BUILD_DIR="${MOSEK_BUILD_DIR}" \
        OUTPUT="${ROOT}/${benchmark}_mosek" \
        sh run-mosek.sh || true
    fi
done

echo
echo "All requested MCNC runs completed under ${ROOT}"
