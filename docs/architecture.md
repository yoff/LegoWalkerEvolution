# Architecture and design decisions

## 1. Keep generation functional; make interchange explicit

F# owns discrete generation, constraint solving, part selection and serialization.
The motion language describes rigid bodies with local attachment sites, passive
hinges, one independent rotational input, signed transmission constraints and
designated foot outputs. Shared leg templates separate a leg's shape from its
mounting arrangement and phase.

`Motion.fs` and `WalkerDesign.fs` define different objects on purpose: an
abstract mechanism is not a physical LEGO assembly. A realization genome
records actual stock parts, placements, pivots, gears, mounts, feet and control.
This keeps kinematic exploration independent of any single leg topology while
retaining an inspectable physical construction.

The CLI emits a versioned program, realization, LDraw model and manifest. LDraw
library support is delegated to the simulator boundary rather than replacing
the F# generator with a mesh-processing language.

## 2. Carry motion, not obsolete hardware or fitness

`motion_carry.py` is the ordinary campaign boundary for imported parents,
contributions and generated offspring. It invokes the current F# `build-walker`
path, preserves the reference body's sampled motion and unit input clock, and
checks electrical speed/polarity, torque limit and payload mode.

The constructor retains useful moving stock and a scaffold; it is not unrestricted
fabrication from equations alone. It reconstructs transmission, payload mounting
and support obligations using bounded catalog-based alternatives. Ground and
motion-space reservations filter routes before exact admission.

Paired brick soles and one/two layers of lower plates are prepared for supported
ankles. Non-brick foot forms are retained with explicit warnings. The ground
contract samples actual sole corners over the prescribed gait, treats only
declared lower sole contacts as feet, and checks other hardware against the
requested margin. Default roll/pitch allowances are zero: the default is a nominal
upright construction requirement, not a guarantee about tilted dynamic poses.

Placement search filters scaffold and pairwise target conflicts before truncating
the candidate set. First-order CoM centering uses estimated component masses and
fixed leg mounts; it does not guarantee an exact final CoM, lower CoM or an optimal
moment of inertia. Explicit layout mutations and repair preferences are preserved.

A failed or timed-out construction never licenses simulation of the old assembly.
Failures and partial output remain visible. Exact-screening repair limits continue
to apply. Explicit immutable targeted/requalification comparisons are separate
experiments, not ordinary legacy fallbacks.

## 3. Independent admission before expensive dynamics

There are deliberately several authorities:

1. **Motion gates** check graph structure, sampled closure and input behavior.
2. **Realization gates** check catalog placements and connector obligations.
3. **Reservation checks** cheaply constrain possible hardware routes.
4. **Simulator reconstruction** independently detects rigid groups, joints and
   transmissions from the exported parts.
5. **Exact static admission** checks initial loading and prescribed motion poses.
6. **Dynamic qualification** measures driven behavior with the declared protocol.

A construction proof or pretty foot trajectory is not walking fitness. Rebuilt
hardware can still fail rigid-body partitioning or collision admission. Prescribed
sampling is not continuous proof.

The adapter records part-local mass preparation, actual payload compounds and
estimated whole-layout inertia. Payload tensors are propagated into both
screening and evaluation diagnostics. Non-payload rigid-group inertia diagnostics
are approximate. LDraw uses Y downward; simulator coordinates map
`[x,y,z] -> [x,-z,-y]` at 0.0004 metres per LDU.

## 4. Make experiments reproducible and budgets explicit

Archive identities distinguish motion, physical assembly and controller/genotype.
Native input files and construction receipts are fingerprinted. A same-genotype
rebuild gets a separate immutable construction binding rather than overwriting
the original birth's source files. A changed genotype creates physical lineage
without inheriting incompatible fitness.

Protocols freeze relevant F#/Python sources, generator binaries, simulator revision,
parts dependencies, payload data and evaluation settings. Campaign updates create
new versions; old outcomes are not rewritten to pretend they used new rules.
Observation reuse requires compatibility.

The SQLite archive records proposals, births, exact checks, cheap simulation
attempts and full trials separately. A short unsafe prefix can reject a candidate
without using a full-trial slot. Once a full trial is reserved, interruption is
still spent work; recovery cannot turn it into free allowance.

Before any ordinary trial reservation, a guard requires a current, immutable
construction receipt and matching genotype. Parallel screening and simulation use
the constructed batch, not a stale canonical candidate with the same DNA.

## 5. Separate local exploration from portable source

The local dashboard owns the campaign registry, ancestry graph, diverse retained
parents and human Workbench contributions. Explicit episode authorization controls
simulation; opening the dashboard is not permission for unlimited compute.

Version control contains source, tests, reusable diagrams and portable seed JSON.
It excludes campaign databases, local graphs, saved trial frames, caches, downloaded
parts and personal photos. Recommended seeds are inputs with concise provenance,
not an archive of historic fitness or this project's development conversation.

The planar production path, experimental spatial composition and legacy four-bar
producer remain explicitly distinguished. Compatibility code does not imply
equivalent guarantees across those paths.
