# Regression tests and reusable illustrations

These are compiler, simulator-boundary, experiment-accounting and UI regressions,
not a committed campaign or a catalogue of certified walking robots.

## Test areas

- `LegoWalkers.Tests/`: typed F# motion constraints, shared leg families, realization,
  catalog connections, construction search, paired feet, ground reservation and mutation.
- `python/`: independent mechanical reconstruction, collision and mass handling,
  motor/contact assumptions, exact screening, protocols, interruption/budget
  accounting, populations, Workbench and portable seed materialization.
- `javascript/`: dashboard selection, pipeline state and motion diagram playback.
- `visualizations/`: reusable project-generated diagrams and rendered illustrations
  used by tests and dashboard explanations. These are not measurements.

See [running and validation](../docs/running.md) for commands, resource-dependent
integration fixtures and interpretation of skipped tests.

## Motion illustrations

The diagrams use solved rigid-body linkage poses rather than hand-drawn foot
animations. Coordinates are LDraw units, with Y downward. Paths and frame diagrams
show kinematics, not buildability, collision clearance or powered walking.

| Fixture | Illustration |
|---|---|
| Four-bar leg | [Solved leg and foot path](visualizations/four-bar.svg) |
| Multi-link Walker1 leg | [Solved linkage](visualizations/walker1.svg) |
| Whole Walker1 fixture | [Whole mechanism](visualizations/walker1-whole-walker.svg) |

`Walker1` is expressed through generic body/site/hinge primitives; it has no
special-case motion solver. Shared leg tests distinguish shape, mounting and phase.
The experimental spatial tests add moving-frame composition without promoting
an unverified spatial robot into the production population.

## Construction and simulation boundaries

Paired ankle tests require real connector support and permit only the lower
plates as ground contacts. One/two-layer preparation is idempotent and preserves
motion and drive. Motion-space tests cover stale/modified plans and non-foot
clearance. Reservation success does not replace independent reconstruction.

`test_motion_carry.py` verifies that old physical inputs cannot obtain an ordinary
trial without a current construction receipt, that same-genotype proof bindings
do not overwrite original inputs, and that failures consume no dynamic allowance.
`test_construction_repairs.py` exercises exact feedback and bounded repairs.

Protocol and observation tests keep implementation changes distinct from compatible
measurements. Trial interruption is spent full-trial work after reservation, not
permission to retry for free. Preview modes must not enter ordinary fitness.

## Portable seed tests

The recommended population is a source fixture: eight motion programs and
realization genomes, explicit hashes, electrical intent and concise provenance.
Tests reject changed inputs, unknown seed IDs and existing destinations. Seed
materialization imports no historic fitness or local graph.
