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
Visible dashboard tabs poll status every two seconds; the larger ancestry forest
refreshes at most every ten seconds unless the result-page revision changes.
Hidden tabs pause live updates and reconnect within ten seconds of becoming visible.
The server shares a ten-second ancestry snapshot across tabs, invalidated early
by a version or result-page change. Ancestry rendering does not hold the episode
control lock; failed snapshot generation is reported rather than cached.
An episode exit code of `-15` records a stopped worker, not a dashboard failure:
interrupted reservations remain spent, and unused authorization is preserved.

Use **Pause episode** before closing the computer. Wait for **Episode paused**:
the dashboard stops the dispatcher and its owned worker/compiler processes,
retains partial files, and records interrupted work without refunding reserved
full trials. Unused authorization is preserved; waking or reopening the dashboard
never resumes automatically. A persisted pause request is completed after a
dashboard restart. Animation rendering has its own separate stop control.

There is no hosted simulation service. Every user constructs their own population,
graph and experiment archive locally. The repository is the software and seed
source, not the original author's running campaign.

Stop workers before changing their runtime environment. Independently launched
workers may survive a dashboard terminal closing. Frozen Python source still
depends on the installed simulator/native dependencies; do not replace those
under active workers. Never run two writers against the same archive.

New worlds use the simulator revision pinned by the current implementation.
Restore that dependency before preparing/resuming a world; do not change the pin
to match an older installed simulator. A simulator revision change prevents reuse
of historical simulation measurements: bug fixes may invalidate both successes
and failures. Retained motion inputs still require current construction and admission.

Campaign runtime migration is an explicit **Prepare runtime** action beside Resume
at the top of the dashboard. It freezes code/settings and
queues fingerprinted retained motion inputs, without re-exporting, rebuilding or
screening walkers. It runs in the background and exposes a preparation log.
Opening the dashboard never migrates a campaign; Resume never migrates the runtime.
Resume is disabled until preparation verifies the current
implementation, including after a dashboard restart or an implementation change.
An unchanged implementation needs only the preparation check.
Successful preparation enables Resume but does not start an episode or authorize
additional trials. Failures retain the previous active version and partial outputs
for diagnosis. A restart records interrupted preparation rather than retrying it.

After Resume, named contribution jobs and candidate planning/building/screening
stages report expensive import, construction and admission work, including attempt
numbers and elapsed startup time. A bounded construction timeout or rejection is
recorded for that walker and does not abort all other inputs. Partial attempts are
retained, not silently rerun. Historical manifests are unsimulated source intent;
only a fresh current construction receipt and normal admission can authorize an
ordinary trial. Explicit targeted requalification keeps its exact-input contract.

The board's **Queued / awaiting processing** column precedes Planning and Building.
It includes candidates awaiting import or construction, not only candidates ready
for screening. The display also corrects the older label/order in frozen snapshots.
Each pipeline entry has one identity and one finite, typed lifecycle stage.
Columns and terminal outcomes are projections of that collection, read from a
consistent database snapshot. Contribution admission hands ownership to the
bound birth; it is not a second board column. Contributions rejected before a
birth exists remain in **Recently finished / rejected**, with their name and
diagnostic. Startup/admission history remains a separate diagnostic view.

Contribution admissions and ordinary candidate screening share the configured
screening-worker capacity, with alternating scheduling when both queues have work.
Ordinary source import, mass estimation and F# construction also run in that pool,
so a slow constructor cannot prevent collecting results or refilling free workers.
An ordinary candidate can reach simulation while contributions are still being
built or checked. Admission workers display their names, phases, PIDs and logs in
the normal pipeline; simulation workers remain separate. Only the scheduler writes
admission receipts or reserves walking trials. Once the episode allowance or cheap
gate limit is reached, in-flight admissions finish, but unscheduled contributions
wait for the next explicitly resumed episode.
Mutation proposal generation runs only when active pipeline work has drained;
it cannot block result collection from an already-running worker.

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

### Watching the builder

Newly prepared runtimes show **Current construction term and constraints** on
building cards. The builder atomically updates `construction-live.json` at most
twice per second (plus initial/final snapshots), including while support routes
are being checked. `construction-events.jsonl` retains compact search events;
the latest full snapshot retains the partial proof and geometric checkpoint.
Counters describe the current physical replay/expansion, not lifetime totals or
a percentage-complete estimate. A snapshot timestamp reveals stale progress.

Side and top views show stock bounding envelopes: fixed/selected geometry, the
current candidate, reserved sampled gait bounds, and reported conflicts.
Projected boxes may overlap without a 3D collision. These are neither full LEGO
renders nor evidence of mesh clearance, walking, or external admission.
The term lists chosen productions, their prerequisites and unresolved goals.
Interrupted/finished workers retain their snapshots, explicitly labeled stopped.

Carry-forward tries up to four centering/layer alternatives, not four identical
retries. Each effective request is fingerprinted with its complete request and
construction/oracle scope; an already-searched identity is skipped before search.
The CLI accepts prior identities through `--seen-requests` (a JSON array).
Existing per-search caches remain scoped to their exact inputs; different
requests do not inherit hard failure clauses merely because they look similar.
No universal cross-request learning is claimed. Success or timeout stops this
obligation, preserving existing accounting and partial-output behavior.

These solver changes require explicit runtime preparation; do not patch a
running version's frozen sources or binaries to enable the visualization.

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
