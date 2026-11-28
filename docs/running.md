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

`--foot-layers 2` increases clearance beneath the ankle brick. Ground plans
explicitly record validated foot stacks: a contact plate, at most one upper
plate, and their aligned 3701 ankle brick on the same output body. The bottom
plate remains the only contact surface; upper stack parts are checked for ground
penetration rather than the non-foot margin. All other parts, including nearby
bricks and ankle pins, retain the requested non-foot margin. This distinction
allows rocking stock soles whose 8 LDU layer spacing projects to less than
8 LDU vertically; it is not an exemption for an entire foot body.
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

Motor placement and drivetrain planes are searched together. The constructor
considers centering the motor housing across the mean foot depth, inward/outward
axial offsets and its original placement, within an explicit 240 LDU axial
tolerance on rebuilt requests. Up to nine motor placements share the existing
32 target-assignment bound, with one assignment reserved per feasible motor
placement. Each selected placement gets compatible input-mesh plane alternatives;
stock axles, supports, housing clearance and exact admission remain mandatory.
This does not add arbitrary lateral motor positions, new shaft centers or new
transmission topology. Already-compound drivetrains retain their existing planes.
Centering is an alternative, not a bypass: retained rails, bridges and walking
stock may rule out every inward position even when a gear route exists.

The scaffold is now a bounded production, not a singleton inherited frame.
Up to twelve alternatives relocate the existing connected backbone upward and,
where clearance permits, along X/Z, then route stock beams and covered friction
pins back to free sockets on the retained walking-interface parts. Walking pivot
positions/axes and linkage motion are unchanged; old frame attachment pins can
be replaced. The inherited scaffold remains a fallback (preferred when all
hardware target tolerances are zero). This is a backbone-relocation/reconnection
grammar, not arbitrary chassis topology synthesis. Every new scaffold still
requires ordinary complete-assembly geometry, connectivity and admission checks.

Before choosing individual supports, the builder forward-checks each bounded
support domain against the selected hardware, gait and bearing axes. Exact
per-part checks are cached by geometry, owner and immutable collision context.
Gait/hardware checks and individual bearing-axis checks have separate cache
entries, so changing one axle does not repeat the complete gait calculation.
Ground checks prepare fixed candidate corners and roll/pitch factors once
instead of rebuilding them for every prescribed gait phase.
Motion-prefix pruning covers the two-beam base case as well as extensions,
before each topology's 512-candidate cutoff. Rejected short routes therefore
cannot consume that bound and force unnecessary three-to-five-beam search.
Base routes stream lazily in the existing length/anchor order, stopping once
the requested motion-clear routes are found instead of materializing and
sorting every geometric candidate.
Each topology also has a 1,024-candidate geometric work bound for its base-pruned
stream. Reaching it enables a legacy-order fallback with a 4,096-candidate work
bound; each stream still admits at most 512 full candidates for final checks.
The additional work bounds prevent deeply blocked prefixes from expanding
hundreds of thousands of partial candidates without producing a complete route.
Fallbacks retain the previous bounded route strategy, but neither stream claims
exhaustive physical coverage.
Actual support domains also include previously selected connector occupancy;
empty-domain explanations retain all decisions affecting that enumeration.
The smallest initial support domains are selected first. Candidate connectors
are checked against remaining support domains before branching: a surviving
cached route supplies a compatibility witness; otherwise the bounded domain
is regenerated with the prospective occupancy. An empty regenerated domain
rejects that candidate, not every possible route or physical construction.
Every selected support records its actual preceding support dependencies.
Scope-bound local nogoods prune matching frontier branches, avoiding unrelated
backtracking when a domain fails independently of those choices. Learned nogoods
can omit axle choices only after checking every joint combination of the freed
finite axle domains. This minimization is limited to 32 counterfactual evaluations
and a two-second cooperative cancellation budget,
once per selected scaffold/walking/hardware/drivetrain/motor-motif family, and only
before any support has been selected. Incomplete checking leaves bindings intact;
individually irrelevant choices are not assumed jointly irrelevant. These extra
propagation checks are not main proof-state visits or dynamic simulation trials.
Previously learned exact, scope-matching clauses can also eliminate an axle
binding when they cover every joint assignment of its finite domain and all
already-freed domains. Coverage reduction uses at most 256 inexpensive clause
checks per failure, without new geometry checks or simulation. Missing coverage
retains the binding; a partial collection of failed values never rejects an
untested value.
Learned nogoods are retained in search checkpoints; they are not transplanted into different
requests or simulator/oracle versions. Live snapshots report learned-conflict
counts alongside checked/rejected parts and cache hits. Compact rejection events
record every failure and its bindings without the visualization throttle.
Only repair-relevant feedback invalidates replay caches; scaffold identity is
included when caching walking repairs. Within each layout family, search remains
depth-first. Family slices grow from 16 to 32, 64 and then 128 visited states,
allowing healthy partial builds more time to complete. At each slice boundary
the search selects a pending branch by
lexicographically least-visited scaffold/walking/hardware/drivetrain/motor-motif
prefixes, retaining cost order for ties. Slices count main state expansions, not
seconds; an expensive geometry expansion cannot be preempted by the scheduler.
The 64-node frontier is an active window, not a deletion cap: overflow branches
are deferred and checkpointed, and participate in slice selection. Pending proof
memory can therefore exceed 64 nodes. This prevents an early axle family from
consuming the entire state budget or dropping the inherited scaffold.
Search is still bounded feasibility search, not globally optimal branch-and-bound.
Exhausting a truncated domain is a failure of this bounded grammar, never a
proof that no physical construction exists.

An optional incremental Z3 backend checks the discrete choices before geometry
expansion: install `python -m pip install '.[solver]'` in the project environment,
then pass `--constraint-solver .venv/bin/z3` to `build-walker`.
F# still generates every production and performs geometry/connectivity checks.
The solver records conditional finite domains from enumerated branches, with
per-goal exclusivity and exact request-scoped conflict clauses. Unsat cores can
exclude a partial family when all registered alternatives are blocked, even if
no existing clause directly matches that partial proof. This is bounded-grammar
inconsistency, not physical impossibility. Repair feedback invalidates registered
domains because repairs can enlarge the production pool.
Unknown results are recorded explicitly and do not prune; solver startup,
protocol and communication failures fail the command rather than falling back.
`constraint-solver.json` records the backend version and executable hash;
live search snapshots report checks, solver prunes and inconclusive warnings.
Standalone CLI builds opt in with that flag. Ordinary campaign construction now
requires the solver extra and supplies the flag automatically. Runtime preparation
hashes and freezes the Z3 executable alongside the compiler; workers use that
frozen copy, and construction receipts include its hash. A missing frozen solver
is an error, never a fallback to a different installed solver.

With that flag, once hardware, motor mounting and axle stock are fixed, F# also
exports a joint support problem. It contains the actual bounded beam/pin route
choices, fully covered scaffold round sockets and their retained connector
occupancy. Connector overlaps and duplicate produced parts forbid incompatible
route pairs; a covered scaffold socket can be occupied by only one route.
The existing rigid-pin bond and path checks also explain locally invalid routes
before a complete assembly is constructed. This information is checked again
against occupancy-regenerated alternatives during physical replay.
Z3 selects one compatible route per support obligation, preferring earlier
ranked alternatives only when the remaining obligations can still be satisfied.
Those assignments guide branching without changing immutable production costs.
Plans and route checks are cached within the exact request context.
Retained moving non-foot parts are also audited against the existing ground plan
before hardware support search. A failure is explained by the fixed
scaffold/walking/drivetrain bindings, not by unrelated payload or support choices.
Validated foot-stack parents use the explicit nonpenetration rule described
above; unregistered or unrelated sole geometry retains the non-foot margin.
The final whole-assembly audit applies the same classification and rejects
forged or stale ground-plan stack metadata.

`joint-support-problem.json` records socket providers, attachment requirements,
route conflicts and local rejection gates. `joint-support-plan.json` and the
event journal record selected assignments or the blocked requirements and their
socket locations. A blocked initial pool or an unknown result is reported, but
does not discard a family: later occupancy can regenerate routes outside that
pool. Such routes remain eligible for normal F# search and validation.
Scaffold choice remains in the outer search; this step allocates the sockets of
the selected scaffold rather than inventing new rails or intermediate anchors.

Motor layout is selected before scaffold search. Flexible builds enumerate driven
shafts at the same signed angular rate as the inherited motor and both axial
facings, preferring shafts near the feet's horizontal center. This reuses the
transmission rerooting rules; the motor phase is set from the solved initial pose,
and the full reference-body phase-equivalence check remains mandatory. Electrical
speed, torque and input clock do not change. The original layout remains available;
zero motor placement tolerance or a non-unit input clock keeps the explicit fixed
shaft and facing.
Each selected layout has its own motion, geometry and support-planning caches.
The construction batch identifies the exported mechanism, including any rerooting;
the original motion remains the request's reference and parent provenance.

When a joint pool's unsat core contains only the two motor support obligations,
the current layout yields to unpenalized pending layouts immediately. The advice
names fixed layout decisions, not axle choices, and is checkpointed separately
from exact nogoods. Its routes remain deferred and can still produce a witness
after other layouts have been tried; no branch is pruned solely by this advice.
Live snapshots report the number of scheduling advisories.
Once a compatible joint assignment guides an available support production, its
family receives the maximum bounded slice (128 states with the normal settings)
to finish local bond/path verification. Motor-only blocked-pool advice can still
yield immediately; compatible progress no longer loses its slice every 16 states.

Construction-only probes of the original shaft-0 Trudge contribution selected
shaft-4 with reversed motor facing at `(280, 0, -70)`. Two sequential runs produced
identical witnesses in 39.6-40.0 seconds and 109 states, with one joint plan and no
rejections. Exact proof replay and whole-assembly ground audits passed. These are
local diagnostic timings, not a campaign performance guarantee or new dynamics
qualification.

Trudge is also a running **symbolic** example. After building the CLI, run:

```sh
.tools/dotnet/dotnet fsi \
  --reference:src/LegoWalkers.Core/bin/Debug/net10.0/LegoWalkers.Core.dll \
  examples/trudge-constraints.fsx .venv/bin/z3
```

This small scaffold/axle/motor-support problem defines two blocked routes and
checks that the solver excludes the scaffold without blaming an irrelevant axle.
It also verifies that an untested third route remains available and that repaired
domains replace old constraints. These are illustrative defined conflicts, not
mined geometry or a reconstruction of the qualified Trudge witness.
The example additionally derives shared/free sockets from a real stock beam:
joint planning reserves the shared socket for the obligation that has no
alternative, and explains why a fixed competing route or retained pin blocks
that allocation. This is a stock-socket subproblem, not a complete Trudge build.
Use this cheap check for constraint-layer progress; compare saved Trudge requests
for states/time to a local build witness, and physical trials separately for
qualification. No solver result or example spends simulation trials.

When source mass estimation succeeds, joint motor/battery/receiver layouts are
ranked before support routing by squared horizontal COM offset, normalized by
the sampled swept-foot half-spans. Battery corrections are recomputed for each
motor/receiver choice instead of balancing only the inherited configuration.
Feasible layouts with better estimated balance lead the existing 32-assignment
search; motor alternatives remain reserved for backtracking. The source mass,
component centers and support bounds are saved in `balance-model.json` and passed
through CLI `--balance-model FILE`. The model is part of the exact request scope.
No model is supplied for layout-preserving repairs or when source mass estimation
fails; the existing explicit diagnostic and geometric search remain in effect.
Measured source scaffold masses adjust the COM estimate for relocated parts
and removed attachment pins before hardware ranking. This is a preference, not
a rejection or qualification: source mass excludes new scaffold/support routes,
and swept-foot bounds are not simultaneous stance contacts.
Preparation compares the compiler's reported construction-policy versions to
the current implementation and rejects a stale CLI instead of freezing newer
source beside older executable behavior. Rebuild the CLI before preparing.

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
