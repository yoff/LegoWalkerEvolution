# LegoWalkerEvolution

Generate LEGO Technic walking robots in **F#**, export real **LDraw assemblies**,
and test them through
[LegoTechnicSimulation](https://github.com/yoff/LegoTechnicSimulation) and MuJoCo.
A local dashboard supports bounded evolutionary search, diverse parents and
human-guided breeding.

This is a research compiler and experiment workbench, not a guarantee of
buildability or successful walking. Kinematic closure, physical reconstruction,
collision admission and dynamic qualification are separate gates.

## Design in one minute

```text
Motion program: rigid bodies, sites, hinges, one input, transmissions, feet
    -> sampled constraint and mobility gates
    -> explicit realization genome and bounded stock-part construction
    -> LDraw + manifest + construction/ground evidence
    -> independent mechanical reconstruction and prescribed-pose admission
    -> bounded powered simulation
    -> immutable observations, diverse parents and new motion proposals
```

Motion and hardware are deliberately separate. Carried designs retain motion
and electrical intent, then use the current bounded constructor before ordinary
campaign screening or simulation. Old assemblies are construction hints, not an
escape hatch when current construction fails. Existing explicit repairs and
layout edits retain their placement preferences.

See [architecture and design decisions](docs/architecture.md),
[running experiments](docs/running.md), and
[recommended seed population](seeds/README.md).

## Setup

- .NET 10 SDK, including F#.
- Python 3.12 recommended; package minimum is 3.10.
- A complete [LDraw parts library](https://library.ldraw.org/library/updates/complete.zip),
  unpacked outside version control.
- Blender is optional for rendered animations; headless simulation needs no Blender.

```sh
git clone https://github.com/yoff/LegoWalkerEvolution.git
cd LegoWalkerEvolution
python3.12 -m venv .venv
.venv/bin/python -m pip install -e '.[dev,physics]'
export LDRAW_LIBRARY=/absolute/path/to/ldraw
dotnet build LegoWalkers.sln
```

The simulator revision and MuJoCo version are pinned in
[pyproject.toml](pyproject.toml). Do not substitute historical simulation results
for measurements under the current protocol.

## Start with the kept population

The repository includes eight portable active kept designs: motion programs,
realization advice, input hashes and concise provenance. It does **not** include
the original campaign graph, database, fitness cache, logs or downloaded parts.

```sh
.venv/bin/python -m legowalkers_sim.seeds --output output/recommended-seeds
sh scripts/dashboard.sh \
  --dotnet "$(command -v dotnet)" \
  --ldraw-library "$LDRAW_LIBRARY" \
  --candidate-batch output/recommended-seeds
```

Open <http://127.0.0.1:8788/>. Starting the dashboard may prepare and statically
screen a version; it does not authorize powered trials. Use the dashboard's
episode action to authorize a bounded number of full trials. Start with low
parallelism if construction or screening is expensive.

To start with only the balanced L:

```sh
.venv/bin/python -m legowalkers_sim.seeds \
  --seed l-balanced --output output/l-seed
```

Use the resulting directory as `--candidate-batch`. Materialization only lowers
the saved source through the current exporter; subsequent ordinary discovery
performs the current hardware rebuild and exact admission.

## Generate and inspect a mechanism

```sh
dotnet run --project src/LegoWalkers.Cli -- realize-walker \
  --example walker1 --feet 4 --steps 96 --output output/example
.venv/bin/python -m legowalkers_sim check \
  output/example/candidate-001/manifest.json
```

The example is a grammar fixture, not a qualified walker. CLI help exposes
motion edits, stock realization, feet, construction feedback and diagnostic
tools. Fresh construction/evaluation destinations must be empty.

## Current capabilities and limits

- Native walkers have **two to six logical feet** driven from one mechanical
  input. Multiple sole contacts do not invent extra feet.
- Supported transverse brick ankles receive paired parallel brick soles and
  stud-mounted plates. Unsupported foot forms remain explicitly diagnosed,
  not silently relabeled as paired feet.
- Hardware/support construction reserves sampled motion space and defaults to
  **8 LDU nominal non-foot ground clearance**. This upright sampled contract is
  not a continuous collision or dynamic roll/pitch proof.
- Estimated CoM centering is a bounded placement preference. Inertia tensors
  and support axes are diagnostics, not a claim of globally optimal mass layout.
- The default powered model is an open-loop 9 V Power Functions XL curve.
  Signed speed preserves polarity. Ideal-servo and collision-disabled previews
  cannot earn ordinary walking fitness.
- Balanced L and Trudge source assemblies passed bounded corrected-simulator
  walking tests. Their reconstructed descendants must qualify afresh; this is
  not hardware certification or floor/timestep convergence.
- Some retained exploratory seeds still fail reconstruction, routing or search
  bounds. They are motion starting points, not promised runnable robots.
- Spatial moving-frame composition exists as a tested experimental layer;
  integrated spatial walker construction/evolution is not production-ready.
- A legacy paired four-bar producer remains for compatibility and regression
  examples. It is not an intermediate stage of native motion compilation.

## Tests

```sh
dotnet test LegoWalkers.sln
.venv/bin/python -m pytest -m 'not integration'
node --test tests/javascript/*.test.cjs
```

Real-parts integration tests require `LDRAW_LIBRARY` and freshly generated
fixtures; see [running and validation](docs/running.md).
The complete baseline is not green; see the explicitly recorded
[validation gaps](docs/validation.md).

## Source map

| Area | Responsibility |
|---|---|
| [F# core](src/LegoWalkers.Core) | Motion constraints, stock parts, realization, construction search and mutation |
| [F# CLI](src/LegoWalkers.Cli) | Deterministic generation and explicit interchange |
| [Python adapter](simulator/legowalkers_sim) | Independent reconstruction, screening, dynamics and experiment orchestration |
| [Tests](tests) | Geometry, protocol, accounting, lineage and dashboard regressions |
| [Scripts](scripts) | Offline planners, previews and bounded diagnostic tools |
| [Seeds](seeds) | Portable recommended motion/realization sources |

Generated campaigns, local populations and reports belong under `output/`,
which is ignored. MIT licensed; see [LICENSE](LICENSE) and
[third-party notices](THIRD_PARTY.md).
