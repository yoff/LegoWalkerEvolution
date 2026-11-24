# Recommended starting population

`recommended/population.json` lists eight active kept source designs. Each entry
contains only a versioned motion program, a realization genome, SHA-256 input
hashes and a concise note. There are no local campaign IDs, ancestry databases,
measurement caches, personal metadata or part meshes.

These are recommended **starting motion ideas**, not eight qualified assemblies:

| ID | Purpose |
|---|---|
| `l-balanced` | Centered battery, paired brick soles and lower plates; source passed a bounded corrected-simulator walking test |
| `trudge-balanced` | Centered/lowered battery and paired plate feet; negative electrical polarity is intentional |
| `outward-l` | Earlier outward-L geometry with retained knee motion |
| `trudge-original` | Earlier Trudge arrangement for diversity |
| `gallop` | Gait/phase alternative; current construction can exhaust its bounded search |
| `four-long` | Long-stride four-bar motion |
| `four-longer` | Longer four-bar variant; rebuilt hardware still needs reconstruction repair |
| `xl-reversed` | Electrical-reversal starting point; polarity is part of the source intent |

The two balanced source assemblies were measured under the corrected simulator:
L advanced approximately 91.29 cm with 15 repeatable cycles; Trudge advanced
approximately 11.43 cm with five. These descriptive observations are **not
imported as fitness**, nor do they certify physical construction or solver
convergence. Trudge's source one-layer nominal non-foot clearance is below the
current 8 LDU reconstruction requirement; it may need a second plate layer or
other current construction changes.

Run:

```sh
.venv/bin/python -m legowalkers_sim.seeds --output output/recommended-seeds
```

Repeat `--seed ID` to select a subset. The output must not already exist. The
materializer verifies hashes, exports each source under the current compiler
and preserves electrical/payload intent. Ordinary discovery then rebuilds
hardware with the current builder and runs exact admission before any powered
trial. Failed seeds remain explicit failed construction ideas, never a fallback
to simulating old hardware.

New measurements, mutations and your kept population belong in your local
ignored experiment directory, not in this portable source snapshot.
