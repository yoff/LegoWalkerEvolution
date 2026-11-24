# Running and validating experiments

Follow the [setup](../README.md#setup) first. Use a fresh output directory for
each construction/evaluation, and keep `LDRAW_LIBRARY` set to the complete local
LDraw library.

## Portable population and dashboard

```sh
.venv/bin/python -m legowalkers_sim.seeds --output output/recommended-seeds
sh scripts/dashboard.sh --dotnet "$(command -v dotnet)" \
  --ldraw-library "$LDRAW_LIBRARY" \
  --candidate-batch output/recommended-seeds
```

`--seed l-balanced` restricts materialization to one portable input. Materialization
runs the current deterministic exporter, not a powered simulation. Each seed
becomes an independent native batch referenced by `seeds.json`; different
mechanisms need not pretend to share one mechanism identity.

The dashboard defaults to loopback port 8788, four simulation workers and two
screening workers. Override with `--simulation-workers 1 --screening-workers 1`
on smaller machines. `--episode-trials` controls the allowance authorized by each
episode action. Static preparation can be expensive even before dynamics.

There is no hosted simulation service. Every user constructs their own population,
graph and experiment archive locally. The repository is the software and seed
source, not the original author's running campaign.

Stop workers before changing their runtime environment. Independently launched
workers may survive a dashboard terminal closing. Frozen Python source still
depends on the installed simulator/native dependencies; do not replace those
under active workers. Never run two writers against the same archive.

Campaign upgrades freeze a new implementation version and preserve accounting.
Recovery records interrupted work explicitly. Do not edit SQLite rows or remove
partial outputs to bypass failed admission or spent full trials.

## Full current construction

```sh
dotnet run --project src/LegoWalkers.Cli -- build-walker \
  --parent output/recommended-seeds/l-balanced/candidate-001 \
  --output output/l-built --limit 1 --steps 96 \
  --foot-policy paired-plates --foot-layers 1 --ground-clearance-ldu 8
```

Use `--foot-layers 2` when one plate layer cannot satisfy the nominal clearance.
`--target-placements FILE` accepts a JSON map from known component IDs to finite
`{"x": ..., "y": ..., "z": ...}` placement preferences in LDU; these preferences
do not bypass routing or change reference motion.

`--foot-policy retain` is an explicit construction diagnostic option, not the
ordinary carry-forward default. It cannot be combined with a two-layer request.

The full build emits `request.json`, search evidence, candidate ground plans and
construction proofs. Construction success still requires exact static admission.
The Python carry-forward path performs bounded centering/layer alternatives and
adds an immutable receipt; running `build-walker` alone is not a campaign receipt.

## Evaluation semantics

The default XL electrical model uses open-loop regulated 9 V, including an
approximately 220 rpm unloaded speed and a conservative 0.4 Nm stall cap.
Signed speed chooses polarity; use physical gearing rather than requesting
impossible motor torque. The motor floats during settling.

Qualification requires the configured free-body contact, closure, tilt, forward
motion and repeatable-cycle checks. Anchored animation, ideal-servo previews and
collision-disabled runs are not ordinary walking fitness. Battery sag, receiver
losses, thermal protection and manufacturing tolerances are not calibrated.

Only permitted feet may support ordinary walkers. Slight contact penetration can
occur in a compliant solver; it must be reported against the declared thresholds,
not hidden or confused with permission for chassis contact. Ground tuning and
timestep are part of the scientific protocol. A single qualified run does not
establish convergence.

## Validation

```sh
dotnet test LegoWalkers.sln
.venv/bin/python -m pytest -m 'not integration'
node --test tests/javascript/*.test.cjs
```

To enable real-library integration fixtures:

```sh
for kind in rigid hinge motor leg walker; do
  dotnet run --project src/LegoWalkers.Cli -- generate \
    --kind "$kind" --output "output/integration-collision/$kind"
done
LEGOWALKERS_CANDIDATES="$PWD/output/integration-collision" \
  .venv/bin/python -m pytest -m integration
```

Tests that require those resources explicitly skip without them. Generated
integration fixtures, build outputs and test reports remain local.

Offline scripts expose additional previews, motif catalogues and packaging
diagnostics. Some intentionally require user-supplied reference files or inputs;
they are not prerequisites for the recommended-seed workflow.
