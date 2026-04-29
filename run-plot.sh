#!/bin/sh
set -e

ROOT="${ROOT:-out}"
PYTHON="${PYTHON:-python3}"
VISUALIZER="${VISUALIZER:-visualize_floorplan.py}"

if [ ! -f "${VISUALIZER}" ]; then
    echo "visualizer not found: ${VISUALIZER}"
    exit 1
fi

if ! "${PYTHON}" -c "import matplotlib" >/dev/null 2>&1; then
    echo "matplotlib is not installed for ${PYTHON}."
    echo "Install it with:"
    echo "  ${PYTHON} -m pip install matplotlib"
    exit 1
fi

if [ "$#" -gt 0 ]; then
    DIRS="$*"
else
    if [ ! -d "${ROOT}" ]; then
        echo "results root not found: ${ROOT}"
        exit 1
    fi
    DIRS=$(find "${ROOT}" -type f -name placements.csv -print | sed 's#/placements.csv##' | sort)
fi

if [ -z "${DIRS}" ]; then
    echo "no result directories found under ${ROOT}"
    exit 1
fi

for dir in ${DIRS}; do
    if [ ! -f "${dir}/summary.json" ]; then
        echo "skipping ${dir}: missing summary.json"
        continue
    fi
    echo "plotting ${dir}"
    "${PYTHON}" "${VISUALIZER}" "${dir}" --output "${dir}/floorplan.png"
done

echo
echo "done"
