# Jansen-style leg: motion fixture

The `jansen` motion example expresses Theo Jansen's canonical planar linkage
with the existing generic rigid-body/site/hinge grammar. It introduces no
Jansen-specific solver, prescribed foot animation or new motor.

```sh
dotnet run --project src/LegoWalkers.Cli -- motion \
  --example jansen --steps 720 --turns 2 --output output/jansen
```

Open the generated `preview.html` to play or scrub the solved leg. `cycle.svg`
shows four crank phases; `program.json` is editable input and `motion.json`
contains solved poses. Outputs are local and ignored by version control.
Both diagrams keep attachment-site connections fixed as the bodies rotate and
show all three edges of each rigid triangle.

## Topology and dimensions

There are seven moving rigid bodies, ten binary hinge constraints and one
grounded chassis. The upper and foot triangles are each one rigid body with
three noncollinear sites, not three independently flexible links. Coincident
multi-body pivots use independent binary hinges with no redundant cycle.

The reference names are crank axle O, fixed pivot Y, crank pin Z, upper rocker X,
lower rocker W, upper knee V, lower knee U and toe T:

| Span | Length |
|---|---:|
| O-Z | 15 |
| Z-X | 50 |
| Z-W | 61.9 |
| Y-X | 41.5 |
| Y-V | 40.1 |
| X-V | 55.8 |
| Y-W | 39.3 |
| V-U | 39.4 |
| W-U | 36.7 |
| U-T | 65.7 |
| W-T | 49 |

Y is at `(-38, +7.8)` relative to O in the project's Y-down convention. The
published ratios are assigned one numerical unit per LDU for this abstract
fixture; they are **not rounded to Technic hole spacing**. The toe radius is
only diagram metadata, not a physical sole.

Circle intersections establish the initial assembly branch only. Subsequent
poses are solved simultaneously by the same constraint solver as the other
examples. Tests cover two turns in both directions, closure, bar dimensions,
branch continuity and independent toe-position references.

## What this establishes, and what it does not

The canonical dimensions produce an approximately 67.9-unit horizontal toe
envelope and 22.5-unit lift. Using a one-unit band above the lowest point as a
purely geometric stance proxy gives roughly 57 units of travel over 43% of
the crank cycle. This is neither measured ground contact nor evidence of walking.

This is an isolated one-leg motion fixture, **not an admitted walker or LEGO
assembly**. The next physical problem is to choose Technic-compatible lengths,
genuinely rigid/braced triangle constructions, crossed-link depth layers and
supported pivots. That assembly would then need independent reconstruction,
collision/ground admission, a complete robot layout and explicitly authorized
powered qualification. The fixture is not automatically added to evolution.

References for the geometric proportions and joint naming:
[Jansen's linkage](https://en.wikipedia.org/wiki/Jansen%27s_linkage) and
[joint/span diagram](https://planex2d.org/docs/examples/jansen-linkage.html).

## Searching stock socket geometry

After building the CLI, run the separate F# diagnostic:

```sh
dotnet fsi scripts/jansen_search.fsx --output output/jansen-stock-search
```

It searches actual catalog socket triples on `32526.dat` L beams and `32271.dat`
angled beams for the upper and foot bodies. This replaces abstract triangles with
one rigid molded part each, rather than assuming an unbraced compound structure.
Straight spans and the crank use stock hole spacing. A bounded dimensional
neighborhood passes through circle-intersection screening, then the generic
constraint solver independently checks shortlisted programs over two turns in
both directions, with 720 samples per turn. Final ranking and acceptance use the
generic solver's foot positions, not the preliminary analytic trajectory.

The declared geometric filters require at least 40 LDU of horizontal envelope,
10-60 LDU of lift, and at least 40 LDU of horizontal low-band travel within 2 LDU
of the lowest point for at least 25% of the cycle. Those are stance proxies, not
walking qualification. Rankings reward low-band travel and duration, and penalize
large changes between five-degree samples; they do not optimize whole-robot mass
or inertia.

The output includes `motion/preview.html`, the editable motion program, a
fingerprinted `search.json`, and `stock-fit.ldr`. The LDraw file is explicitly an
**exploded socket-fit study**, with no pins, motor, axial retention or collision
admission. Individual socket fits do not yet establish a complete assemblable
leg: spare beam ends, crossed links, shared-pivot stack thickness and foot
attachment still require construction and independent admission.
