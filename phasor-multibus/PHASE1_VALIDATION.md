# Phase 1 — solver + topology, validated against pandapower

**Status:** COMPLETE (14 Jul 2026). The in-browser Newton-Raphson power flow
(`powerflow.js`) reproduces an independent pandapower solve of the same network
to **machine precision** (max |ΔV| ≈ 5×10⁻¹⁵ pu) across all scenarios, for both
the fixed-tap core and the OLTC outer loop. Numbers are right; the D3 layer
(Phase 2) can now be built on top.

This is the "AC solve" that `clients/ea/power-factor/lib/transit_ladder.py`
explicitly deferred (it does a linearised voltage-drop walk; this does the full
Newton-Raphson it calls "the ACPF Phase 2 step").

---

## 1. The model

A **single equivalent GXP feeder**: one representative radial path from the grid
down to the customer LV bus, 220 kV → 400/230 V, with each level's parallel
plant aggregated so the one string carries a realistic GXP's worth of load and
cable charging.

```
 GRID ──X_grid── B1 ──220kV line── B2 ══220/33 OLTC══ B3 ──33kV cable── B4
 (slack)  (Thév)  220kV            220kV   (reg B3→1.03)  33kV           33kV
                  [STATCOM/                                                │
                   reactor]                                       ══33/11 OLTC══
                                                                  (reg B5→1.02)
   B8 ──LV feeder── B7 ══11/0.4 fixed══ B6 ──11kV feeder── B5
   230V  (R-domin)  400V   (+2.5% boost)  11kV              11kV
  [load + LV C]
```

Bus 1–8 are the brief's cascade. **GRID** is a hidden ideal slack sitting behind
a **grid Thevenin reactance** (system short-circuit strength, ~2.5 GVA fault
level → X≈0.04 pu). This is essential: it is what lets the 220 kV bus (B1)
*respond* to reactive injection/absorption. Without it, a slack at B1 would pin
the 220 kV voltage and the STATCOM demonstration ("the 220 kV comes down")
could not move.

**Aggregation tree** (from `transit_ladder.py`, DH-reviewed 13 Jul 2026):
4 zone subs × 4 feeders/zone × 6 DTx/feeder = 96 DTx ≈ 48 MVA of distribution
capacity → a representative ~40 MVA GXP. For N parallel branches at a level:
series Z ÷ N, shunt charging × N. This is a standard equivalent-feeder
reduction — **not** a literal single 500 kVA transformer.

### Per-unit convention
Common system base **100 MVA / 50 Hz**. Each bus carries per-unit voltage on its
**own** kV base (220 / 33 / 11 / 0.4 kV), so 1.0 pu at every bus means that
bus's nominal kV. The transformer off-nominal tap ratio therefore carries only
the *deviation* from nominal turns ratio; the nominal ratio is absorbed by the
per-bus bases. Loads are PQ (drawn from the network); a leading (capacitive)
customer is Q < 0. Shunt Bs > 0 injects vars (capacitive); Bs < 0 absorbs
(reactor / STATCOM-absorb).

---

## 2. Parameters and sources

All values are in `params.json` with a `source` note on every element. Headline
transmission values are **found** in the ac-powerflow PSS/E model (real
Albany–Silverdale 220 kV line, SVL/ALB 220/33 transformers); distribution values
are the DH-reviewed `transit_ladder.py` chain + `research/distribution-kb` +
standard NZ/IEC cable/transformer data (in-repo `[I]` = representative).

### The reactance → resistance flip (of the conductor segments)

| Line/cable level | R (pu) | X (pu) | X/R | Regime |
|---|---|---|---|---|
| grid Thevenin | 0.004 | 0.040 | 10.0 | strongly X-dominated |
| 220 kV line (GXP incomer) | 0.002 | 0.015 | 7.5 | strongly X-dominated |
| 33 kV cable | 0.011 | 0.0203 | 1.84 | X-dominated |
| 11 kV feeder | 0.0313 | 0.0188 | **0.60** | **R-dominated (the flip)** |
| LV feeder | 0.040 | 0.012 | 0.30 | strongly R-dominated |

The lever that works at 220 kV (reactive power against X) is the wrong lever at
400 V (voltage responds to real power against R). The flip lands between 33 kV
and 11 kV.

Two honesty notes (from the review): (1) this is a property of the **conductor
segments**, and X/R of a passive element is an **input** (X/R is invariant under
the parallel-N aggregation — ÷N cancels), not something the solve discovers. (2)
The three transformers interleaved in the same cascade stay strongly X-dominated
(220/33 X/R≈25, 33/11 ≈27, 11/0.4 DTx ≈4), so the true bus-to-bus series
impedance is **non-monotonic** — the flip is about the lines/cables/feeders.

### Which way does the leading current grow? (a correction to the brief)

The brief (BRIEF.md §2/§6) says the phasor "should show the current leading more
as you **descend**." The validated solve shows the **opposite**, and the opposite
is the honest, stronger story. In the `overnight` case the branch reactive flow
`Q_from` (−ve = flowing **up** toward transmission) and the leading angle grow as
you go **up**:

| branch (top→bottom) | P (MW) | Q (MVAr, −=up) | lead angle |
|---|---|---|---|
| 220 kV line | 18.5 | **−14.3** | **−37.8°** |
| 220/33 tx | 18.5 | −12.5 | −34.1° |
| 33 kV cable | 18.5 | −13.0 | −35.2° |
| 33/11 tx | 18.4 | −11.0 | −30.9° |
| 11 kV feeder | 18.4 | −11.2 | −31.4° |
| 11/0.4 DTx | 18.3 | −10.7 | −30.3° |
| LV feeder | 18.2 | −11.1 | −31.3° |

The distributed cable charging **accumulates upward**: each level's shunt C adds
leading vars that transit toward transmission, so the leading angle is *largest*
at the 220 kV bus (−37.8°) and smallest at the customer end. This is exactly the
SVL-canary / transit-ladder thesis — the leading condition is worst where all the
downstream charging piles up. **Phase 2 must orient the phasor scenes to show
leading growing UP the cascade, not down.**

---

## 3. The solver (`powerflow.js`)

Self-contained, no dependencies, runs in the browser (`window.PowerFlow`) and
Node (`module.exports`) — the same code that draws the picture is the code that
was validated.

- **`solveNR(net)`** — polar Newton-Raphson. Dense Ybus with line pi-sections,
  off-nominal tap transformers (tap on the HV/from side, MATPOWER convention:
  `Yff=(y+jb/2)/t²`, `Yft=Ytf=−y/t`, `Ytt=y+jb/2`; so `V_lv = V_hv/t` and `t<1`
  boosts), bus shunts, slack/PV/PQ, full analytic Jacobian, Gaussian-elimination
  linear solve. Converges in ~4 iterations from a flat start.
- **`solve(net)`** — wraps `solveNR` in an **OLTC outer loop**: each OLTC
  transformer steps its discrete tap (±1.25% steps, finite range) toward holding
  its controlled LV bus within a deadband of target; a changer that wants to move
  further but is at its limit is flagged `tappedOut`. Raising the HV tap lowers
  the LV bus.

---

## 4. Validation method and result

`reference/pandapower_ref.py` rebuilds the **same** `params.json` network with
pandapower's own line / transformer / Newton-Raphson models — an *independent*
implementation of the tap convention (pandapower 3.4, `tap_changer_type='Ratio'`,
`tap_side='hv'`). `reference/validate.js` drives both and compares.

- **Stage A — core NR** (all OLTC taps neutral, DTx fixed): validates the Ybus
  (lines + tap transformers + shunts), the NR core, **and the branch-flow
  expressions** (`S = V·conj(I)` with the tap stamp) — bus V, angle *and* the
  per-branch P/Q at both ends are compared.
- **Stage B — OLTC loop**: run the JS OLTC outer loop, fix the tap steps it picks
  in pandapower, confirm voltages+flows still match (the tap point is a true PF
  solution), **and assert** each regulated bus is within deadband (or tapped out)
  and `res.tapConverged` is true.
- **Stage C — independent tap selection**: pandapower's **own**
  `DiscreteTapControl` picks the taps from scratch; assert its regulated buses
  land in band and its selected steps agree with the JS controller (±1). This
  validates the tap-*selection* logic, not just the operating point. (This stage
  caught a real controller bug during the review — a wrong-direction tap freeze —
  which is now fixed; JS and pandapower now select identical taps.)
- **Stage D — tap-out**: force an OLTC to its limit; assert powerflow.js flags
  `tappedOut` and the escaped bus leaves the band, and cross-check the clamped
  point against pandapower.

```
Stage A (V/angle/flows) ....... all 5 scenarios  dVm≤1e-13  dFlow=0     OK
Stage B (+deadband, tapConv) .. all 5 scenarios  dVm≤5e-14  dFlow=0     OK
Stage C (tap selection) ....... JS steps == pandapower DiscreteTapControl steps, all in band
Stage D (tap-out) ............. tappedOut=max, bus escapes band, matches pandapower
RESULT: PASS — Vm<1e-9 pu, flows<1e-6 MVA (gates are regression guards; the
achieved floor is ~5e-15 pu).
```

Run it: `node reference/validate.js` (uses the gridlytics venv python with
pandapower, path set inside the script).

---

## 5. The demonstrations (from the validated solve)

Per-unit voltage staircase (OLTC solution):

| scen | B1 | B2 | B3 | B4 | B5 | B6 | B7 | B8 |
|---|---|---|---|---|---|---|---|---|
| day_peak | 0.993 | 0.991 | 1.028 | 1.021 | 1.026 | 1.012 | 1.018 | 1.001 |
| overnight | 1.005 | 1.007 | 1.031 | 1.031 | 1.024 | 1.021 | 1.052 | **1.047** |
| overnight_x | 1.007 | 1.009 | 1.025 | 1.027 | 1.022 | 1.021 | 1.058 | **1.055** |
| statcom_demo | **0.966** | 0.968 | 1.031 | 1.031 | 1.024 | 1.020 | 1.052 | **1.047** |

1. **"Transpower can't fix this from the 220 kV bus."** `overnight` → `statcom_demo`
   adds a reactor at B1 (B=−1.0 pu = 100 MVAr nominal, ~93 MVAr actual at the
   depressed bus): the 220 kV bus falls **1.005 → 0.966** (−4%), but the customer
   LV (B8) is unchanged (1.0467 → 1.0465) because the 220/33 OLTC re-taps from
   −1 to −4 steps to hold B3 at 1.03. The var device is on the wrong side of a
   servo that cancels it.
2. **The LV floats up.** In the leading overnight cases B8 sits at 1.047–1.055
   (241–243 V) even though the OLTCs hold B3/B5 — because below the last OLTC
   there is only the R-dominated feeder + the fixed **+2.5 % DTx boost** (designed
   for peak *under*voltage, it *worsens* overnight overvoltage) + the LV feeder.
   This is `transit_ladder`'s "residual below OLTC" made explicit.
3. **Tap-out.** Push far enough (or narrow the range) and an OLTC hits its limit,
   `tappedOut='max'`, and its regulated bus escapes the deadband and floats. This
   is exercised and cross-checked against pandapower in Stage D of the validation
   (not one of the five physical scenarios, which all stay in-range).

---

## 6. Honest caveats

- pandapower validates the **solver** (voltages, angles, branch flows, tap
  selection, tap-out), not the **realism of the parameters** (both sides read the
  same `params.json`). Parameter honesty rests on the ac-powerflow model
  (transmission, found) and the DH-reviewed `transit_ladder` chain +
  distribution-kb + standard tables (distribution, representative `[I]`). The LV
  feeder is the least directly-auditable element (representative, order-of-
  magnitude — see its `params.json` source note).
- v1 is a **single radial string**. The mesh option (two GXPs feeding a tied
  distribution network — the parallel-path / cluster-end point from
  `two_bus_model.md` §6) is Phase 4.
- The STATCOM/reactor is modelled as a **fixed shunt B** at B1 for Phase 1. A
  V-regulating (PV-like) STATCOM with ±B limits and saturation is Phase 3. The
  solver's PV-bus path exists and is correct on a slack/PV/PQ test, but has **no
  reactive-limit (PV→PQ) switching** and is not yet exercised by the validation
  suite — it is Phase-3 scope.
- Aggregation assumes identical parallel branches at each level. Fine for a
  pedagogical equivalent; not a substitute for a real network case.

## 7. Files

| File | What |
|---|---|
| `params.json` | Auditable 8-bus topology, per-unit, source note per element |
| `powerflow.js` | NR solver + OLTC loop (browser + Node) |
| `reference/pandapower_ref.py` | Independent pandapower reference (neutral / taps / autotap modes) from the same params |
| `reference/validate.js` | Cross-validation driver (Stages A–D) |
| `PHASE1_VALIDATION.md` | This document |

`reference/js_taps*.json` are generated by `validate.js` (git-ignored).
