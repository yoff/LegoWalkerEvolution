# Validation status and known gaps

The initial source publication is not a claim that every regression or retained
seed passes. No powered trials were authorized for repository preparation.

## Verified

- The F# CLI builds without warnings or errors.
- The complete F# suite passes: **231 tests**.
- The focused Python seed, carry-forward, repair, native discovery, screening,
  parallel accounting, upgrade and Workbench suite passes:
  **210 passed, 6 skipped**.
- The reused-founder fairness regression passes with the current mock constructor
  and immutable construction bindings.
- All **eight portable sources** export with the current compiler and satisfy
  the existing multi-batch native seed-pool contract.
- Project-generated models and seeds contain no bundled LDraw part meshes.

## Known baseline failures

The broad Python run reported 971 passes, five failures, one skip and 27 deselections.
Two failures were current-construction fixture/dispatch integration defects; both
were fixed and rerun successfully in the focused suites above. The remaining
unresolved failures are:

- The two `none` cases of
  `test_optional_parser_cache_proof_is_exact_and_preserves_snapshot_verification`
  in `tests/python/test_observations.py`: the compatibility proof returns false
  where those fixtures expect true. Do not loosen observation compatibility to
  force reuse without investigating the source-equivalence obligation.
- `test_whole_walker_loop_replays_and_reconstructs_without_stepping` in
  `tests/python/test_whole_walker_loop.py`: the offline F# witness subprocess
  exits unsuccessfully. This test also needs a local SDK, compiled CLI and
  actual parts library; do not interpret a missing-resource environment as a
  completed physical regression.

The JavaScript suite reports **23 passed, 5 failed**. The unresolved cases are
the top-view alignment, coincident-foot depth separation, projection/playback
state, visibility recovery and solved-3D-coordinate cases in
`tests/javascript/motion-diagrams.test.cjs`. These expectations need separate
investigation; they are not suppressed or relabeled as passing tests.

## Construction limits, separate from test results

- The longer four-bar source lowers through the current ground-aware constructor,
  but independent admission rejects a split crank/shaft rigid-body partition.
  Lowering success is not a simulator-ready model.
- Gallop's current centered-placement construction exceeded its 600-second
  per-attempt limit. Partial output is preserved and the failure is explicit;
  old hardware is not substituted.
- The balanced Trudge source's original plate stack did not meet the v0078
  8 LDU margin applied to every non-contact part. A complete-gait audit of the
  saved attempt-001 request finds a minimum of about 6.90 LDU beneath each ankle
  brick: the lower plates are correctly registered as contacts, but rocking
  projects their 8 LDU spacing to a smaller vertical gap. Adding a second layer
  transfers that violation to the upper plates rather than fixing it. Development
  ground plans now recognize the validated stock foot stack explicitly: upper
  stack parts must not penetrate ground, while other hardware retains the 8 LDU
  margin. With those rules and joint support planning, the saved ground-constrained
  request produced a complete construction witness in 219 states, about 64 seconds,
  with no rejected candidates. Its exact proof replay and whole-assembly ground
  audit also passed. No new physics trials were run. Frozen runtimes retain their
  original rules until explicitly upgraded; a newly built descendant must still
  qualify independently.

Those limitations are why the population is described as recommended **motion
starting points**, not universally qualified current assemblies.
