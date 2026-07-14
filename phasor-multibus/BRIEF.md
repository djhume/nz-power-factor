# phasor-multibus - extending the two-bus phasor tool to a full 220 kV-to-230 V cascade (project brief)

**Created:** 14 July 2026. **Hat:** Gridlytics / public physics (generic, representative values; no EA / Silverdale-forensic specifics). **Home:** this repo, `github.com/djhume/nz-power-factor`, alongside the original at `phasor/` (live on the same GitHub Pages site). **Feeds:** the flagship power-factor paper (`research/overvoltage-flagship/`) and the load-model reform programme.

**Start a fresh session scoped to this.** The Silverdale forensic context is saved separately and is not needed here.

---

## 0. Build ON the original, do not restart

Dave already built and presented (at the Feb 2026 EA Reactive Power Workshop) a **two-bus voltage phasor tool**. It lives here:
- Local: `tools/public/nz-power-factor/phasor/index.html` (self-contained, ~2,181 lines, D3 v7, no build step)
- GitHub: `github.com/djhume/nz-power-factor` (folder `phasor/`)
- **Live: https://djhume.github.io/nz-power-factor/phasor/**

What it already does, and what carries straight over:
- Solves the two-bus system **analytically** (`solveTwoBus`; `solveStatcomB` sizes the STATCOM susceptance). Exact for two buses, so there is **no Newton-Raphson** - that is the one piece that must change to go multi-bus.
- Frequency-dependent parameters / the f-V coupling (`getEffParams`, `vrAtFreq`, `setFreq`).
- Cable charging, and STATCOM and shunt-reactor devices (`toggleDevice`, `setDeviceStates`, `updateStatcomDisplay`).
- A scripted **6-scene "story" mode** (`playStory`/`pauseStory`/`stopStory`, `showAnnotation`), a **presentation mode** (`togglePresentation`), and **SVG export** (`toSVG`).
- All the phasor/circuit drawing (`drawCircuit`, `drawChainedCurrent`, `drawReference`, `drawGround`, `updateCircShuntArrow`, `describeArc`, `rot`, `perpOffset`).

**First task of the build session: read `../phasor/index.html` in full.** Reuse its D3 rendering, device toggles, and story/presentation machinery. Replace only the model layer (two-bus analytic -> N-bus power flow) and add the transformer/tap-changer/LV chain.

## 1. What the extension adds, and the one thing it must prove

Take the two-bus picture down the whole cascade: 220 kilovolt (kV) to 230 volt, in per unit, with the distribution transformers and their on-load tap changers (OLTCs), and a real power flow solving underneath.

**The load-bearing demonstration** (a new scene): absorb reactive power with a STATCOM / shunt reactor on the 220 kV bus, watch the 220 kV voltage come down, and watch the distribution voltage barely move - because the OLTCs underneath re-tap to hold their low-voltage targets. The var device is on the wrong side of a servo that cancels it. Then push load lighter / cabling heavier until the OLTCs tap out, and the low voltage floats up regardless. That is "Transpower cannot fix this from the 220 kV bus", made visible - the natural next scene after the existing tool ends at shunt reactors.

## 2. The physics to get right (so it is honest, not a cartoon)

1. **The tap changer decouples the low-voltage bus from the high-voltage bus.** The OLTC holds its low-voltage side at a target by changing the turns ratio, so the low-voltage voltage is set by the OLTC target, not the upstream voltage. That is the mechanism that defeats transmission-level var control. (The OLTC also shifts leakage reactance X slightly across the tap range, a few percent up to ~10-15%, often asymmetric - model the tap as a ratio with fixed X for v1, add an X-vs-tap curve later. The ratio is the point.)
2. **The resistance-to-reactance ratio flips going down.** Transmission is reactance-dominated (voltage responds to reactive power - var control works up top); low voltage is resistance-dominated (voltage responds to real power). The lever that works at 220 kV is the wrong lever at 400 V. The phasor view should show the current leading more as you descend.
3. **Cable shunt capacitance is the var source.** The cables generate reactive power (proportional to omega*C*V^2) flowing UP toward transmission - the "load went leading" story. Distributed shunt capacitance at each level, not a lumped load.
4. **Tap range and deadband are finite.** OLTCs +/-10 to +/-16% in discrete steps, with a deadband. Tap-out is a first-class state to visualise.

## 3. The model (~6-8 buses, radial for v1)

| Bus | Level | Element to next bus | Notes |
|---|---|---|---|
| 1 | 220 kV | slack (the grid) | STATCOM / reactor toggle + MVAr slider here |
| 2 | 220 kV | 220 kV line (short) | the GXP incomer |
| 3 | 33 kV | 220/33 transformer, **OLTC** | regulates bus 3 |
| 4 | 33 kV | 33 kV cable (shunt C) | sub-transmission |
| 5 | 11 kV | 33/11 transformer, **OLTC** | zone substation, regulates bus 5 |
| 6 | 11 kV | 11 kV cable/line (shunt C) | distribution feeder |
| 7 | 400 V | 11/0.4 transformer, fixed tap | distribution transformer |
| 8 | 230 V | LV feeder (R-dominant) + load + shunt C | customer end |

Loads as PQ (day/night presets); slack at bus 1. Later: a small mesh option (two GXPs feeding a tied distribution network) for the parallel-path point; v1 is the single radial string.

## 4. Realistic parameters (do this properly)

- **220 kV line + 220/33 transformer X/R:** the ac-powerflow PSS/E model - `github.com/djhume/ac-powerflow` (private) / local `tools/models/ac-powerflow/` - real SVL/ALB values.
- **33 kV and 11 kV cables (R, X, shunt C per km + lengths):** standard NZ cable tables + the distribution knowledge base (`research/distribution-kb/`). Get the shunt C right; it is the var source.
- **33/11 and 11/0.4 transformer impedances:** typical (33/11 ~5-8% X; 11/0.4 ~4-5% X), realistic X/R.
- **The R/X flip:** tabulate X/R at each level explicitly (a headline output, not just an input).
- **Tap ranges:** OLTC +/-10 to +/-16% in ~1.25% steps; 11/0.4 usually an off-load fixed tap.
- Keep values in one auditable `params.js`/`.json` with a source note each.

## 5. The solver

Newton-Raphson power flow, **in-browser (JavaScript)** to keep it a self-contained static site like the original. Tiny system (6-10 buses): bus admittance matrix with transformer off-nominal taps and shunt admittances, PV/PQ/slack, plus a simple outer loop stepping the OLTC taps to hold each regulated bus within its deadband. A few hundred lines. **Validate against a Python reference** (pandapower or the ac-powerflow model) on a test case before trusting the visual - numbers right first, then draw.

## 6. The visualisation (extend the original's D3)

Reuse the original's phasor/circuit drawing and add:
- **One-line diagram** of the cascade: buses, transformers with live tap indicators, cables, the 220 kV device toggle; colour buses by per-unit voltage.
- **Voltage-profile "staircase"**: per-unit voltage across levels - the low voltage floating up while the 220 kV is held down.
- **Phasor diagram** at a selected bus, showing the leading current / power factor growing as you descend (reuse `drawChainedCurrent` etc.).
- **Reactive-power-flow arrows** up from the cables.
- **New scenes** in the existing story machinery: (a) STATCOM-cannot-fix-it; (b) tap-out; (c) the R/X flip and reverse-Q. Keep presentation mode + SVG export.

## 7. Phasing

0. Read `../phasor/index.html`; map the reusable rendering + story machinery.
1. Swap the model layer: JS Newton-Raphson solver + the radial 220-to-400 V topology with realistic params; validate vs Python.
2. Extend the D3: one-line + voltage-staircase + multi-bus phasors on a fixed scenario.
3. Interaction + new scenes (the three demonstrations) in the existing story mode.
4. Polish + extend: mesh option, X-vs-tap refinement, embed-ready for the paper/site.

## 8. Decisions to confirm at kick-off

- New subfolder in this repo (`phasor-multibus/`, assumed) versus upgrading `phasor/` in place - keep the original intact, add alongside.
- In-browser JS solver (recommended) vs Python backend + precompute.
- Radial only for v1, or include the small mesh from the start.
- Exact bus count / levels (the table above is a start).

## 9. Pointers

- **The original tool:** `../phasor/index.html` (this repo) / https://djhume.github.io/nz-power-factor/phasor/
- Physics entry point: `clients/ea/over-freq-event-19-May-2026/svl_pf_analysis/two_bus_model.md`.
- Transmission parameters: `github.com/djhume/ac-powerflow` / `tools/models/ac-powerflow/`.
- Distribution parameters: `research/distribution-kb/`.
- The argument it serves: `research/overvoltage-flagship/` ("When the load went leading") and the load-model reform programme.
