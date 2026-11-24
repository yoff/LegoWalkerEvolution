#!/bin/sh
set -eu

project=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$project"

if [ ! -x .venv/bin/python ]; then
    printf '%s\n' "ERROR: Missing project Python environment: $project/.venv/bin/python" \
        "Follow the setup instructions in $project/README.md before starting the dashboard." >&2
    exit 1
fi

exec .venv/bin/python -u -m legowalkers_sim.dashboard \
    --campaign output/walker-discovery \
    --output output/native-discovery \
    --candidate-batch output/recommended-seeds \
    --discover --cycle-aware --episode-trials 4 --port 8788 \
    --simulation-workers 4 \
    --screening-workers 2 \
    "$@"
