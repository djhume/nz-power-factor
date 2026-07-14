#!/usr/bin/env python3
"""
pandapower_ref.py — independent reference power flow for the multi-bus phasor
tool, built from the SAME params.json with pandapower's own line / transformer /
Newton-Raphson (and tap-control) models. Used to validate powerflow.js.

The transformer tap convention here is pandapower's (independent of the JS tap
model): a transformer off-nominal ratio t on the HV/from side maps to pandapower
tap_pos = round((t-1)/0.0125), tap_step_percent=1.25, tap_side='hv',
tap_changer_type='Ratio'. Lines convert pu -> ohms/nF on the FROM bus's kV base.

Every case dumps per-bus vm/va AND per-branch flows (p_from/q_from/p_to/q_to in
MW/MVAr, from=hv for transformers) so the JS branch-flow expressions are checked
too, not just voltages.

Modes:
  neutral            all scenarios, OLTC taps at t=1 (fixed DTx from params)
  taps  <file.json>  taps from a JSON of {case: {branch_id: steps, ...}}; a case
                     may include "_scenario": {P,Q,deviceBs} to override the load
                     (else the case name is looked up in params.scenarios)
  autotap            all scenarios, pandapower's OWN DiscreteTapControl selects
                     the OLTC taps (independent check of the tap-SELECTION logic)
Writes a JSON dump to stdout.
"""
import json
import math
import sys
from pathlib import Path

import pandapower as pp
from pandapower.control import DiscreteTapControl

HERE = Path(__file__).resolve().parent
PARAMS = json.loads((HERE.parent / "params.json").read_text())
BASE = PARAMS["meta"]["baseMVA"]
F0 = PARAMS["meta"]["f0_Hz"]
STEP = 0.0125  # 1.25% tap step (uniform mapping)

BUS_KV = {b["id"]: b["kV"] for b in PARAMS["buses"]}


def build_net(scenario, oltc_steps, autotap=False):
    """scenario: {P, Q (load at B8), deviceBs (pu at B1)}.
    oltc_steps: {branch_id: int steps} used when autotap is False.
    Returns (net, bus_ix, br_ix) with br_ix[id] = ("line"|"trafo", elem_index)."""
    net = pp.create_empty_network(sn_mva=BASE, f_hz=F0)
    bus_ix, br_ix = {}, {}
    for b in PARAMS["buses"]:
        bus_ix[b["id"]] = pp.create_bus(net, vn_kv=b["kV"], name=b["id"])
    for b in PARAMS["buses"]:
        if b["type"] == "slack":
            pp.create_ext_grid(net, bus_ix[b["id"]], vm_pu=b.get("Vset", 1.0),
                               va_degree=0.0)
    for b in PARAMS["buses"]:
        bs = b.get("Bs", 0.0) or 0.0
        if b["id"] == "B1":
            bs = scenario.get("deviceBs", 0.0) or 0.0
        if abs(bs) > 0:  # pu Bs>0 injects vars -> pandapower shunt q_mvar<0
            pp.create_shunt(net, bus_ix[b["id"]], q_mvar=-bs * BASE, p_mw=0.0)
    pp.create_load(net, bus_ix["B8"], p_mw=scenario["P"], q_mvar=scenario["Q"])

    for br in PARAMS["branches"]:
        f, t = br["from"], br["to"]
        r, x, bsh = br.get("r", 0.0), br.get("x", 0.0), br.get("b", 0.0)
        if br["type"] == "line":
            from_kv = BUS_KV[f]
            zbase = (from_kv ** 2) / BASE
            ybase = BASE / (from_kv ** 2)
            c_nf = (bsh * ybase) / (2 * math.pi * F0) * 1e9 if bsh else 0.0
            lid = pp.create_line_from_parameters(
                net, bus_ix[f], bus_ix[t], length_km=1.0,
                r_ohm_per_km=r * zbase, x_ohm_per_km=x * zbase,
                c_nf_per_km=c_nf, max_i_ka=10.0, name=br["id"])
            br_ix[br["id"]] = ("line", lid)
        else:
            from_kv, to_kv = BUS_KV[f], BUS_KV[t]
            z_pu = math.hypot(r, x)
            if br.get("oltc") and not autotap:
                steps = oltc_steps.get(br["id"], 0)
            elif not br.get("oltc"):
                steps = round((br.get("tap", 1.0) - 1.0) / STEP)  # fixed DTx
            else:
                steps = 0  # autotap: controller will move it
            tid = pp.create_transformer_from_parameters(
                net, hv_bus=bus_ix[f], lv_bus=bus_ix[t], sn_mva=BASE,
                vn_hv_kv=from_kv, vn_lv_kv=to_kv,
                vk_percent=z_pu * 100.0, vkr_percent=r * 100.0,
                pfe_kw=0.0, i0_percent=0.0,
                tap_side="hv", tap_neutral=0, tap_step_percent=STEP * 100.0,
                tap_pos=steps, tap_min=-40, tap_max=40,
                tap_changer_type="Ratio", name=br["id"])
            br_ix[br["id"]] = ("trafo", tid)
            if autotap and br.get("oltc"):
                o = br["oltc"]
                DiscreteTapControl(net, tid, side="lv",
                                   vm_lower_pu=o["vset"] - o["band"],
                                   vm_upper_pu=o["vset"] + o["band"])
    return net, bus_ix, br_ix


def run_case(scenario, oltc_steps, autotap=False):
    net, bus_ix, br_ix = build_net(scenario, oltc_steps, autotap=autotap)
    pp.runpp(net, algorithm="nr", calculate_voltage_angles=True, init="flat",
             tolerance_mva=1e-10, max_iteration=50, run_control=autotap)
    out = {"buses": {}, "branches": {}, "taps": {},
           "converged": bool(net["converged"])}
    for bid, ix in bus_ix.items():
        out["buses"][bid] = {"vm": float(net.res_bus.vm_pu.at[ix]),
                             "va": float(net.res_bus.va_degree.at[ix])}
    for bid, (kind, ix) in br_ix.items():
        if kind == "line":
            r = net.res_line
            out["branches"][bid] = {
                "Pfrom": float(r.p_from_mw.at[ix]), "Qfrom": float(r.q_from_mvar.at[ix]),
                "Pto": float(r.p_to_mw.at[ix]), "Qto": float(r.q_to_mvar.at[ix])}
        else:
            r = net.res_trafo
            out["branches"][bid] = {  # hv = from, lv = to
                "Pfrom": float(r.p_hv_mw.at[ix]), "Qfrom": float(r.q_hv_mvar.at[ix]),
                "Pto": float(r.p_lv_mw.at[ix]), "Qto": float(r.q_lv_mvar.at[ix])}
            if PARAMS_BR_OLTC.get(bid):
                out["taps"][bid] = int(net.trafo.tap_pos.at[ix])
    return out


PARAMS_BR_OLTC = {br["id"]: bool(br.get("oltc")) for br in PARAMS["branches"]}


def named_scenarios():
    return [(n, s) for n, s in PARAMS["scenarios"].items()
            if not n.startswith("_") and isinstance(s, dict) and "P" in s]


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "neutral"
    dump = {}
    if mode == "taps":
        cases = json.loads(Path(sys.argv[2]).read_text())
        for name, val in cases.items():
            scen = val.get("_scenario") or dict(PARAMS["scenarios"][name])
            steps = {k: int(v) for k, v in val.items() if not k.startswith("_")}
            dump[name] = run_case(scen, steps)
    elif mode == "autotap":
        for name, scen in named_scenarios():
            dump[name] = run_case(scen, {}, autotap=True)
    else:  # neutral
        for name, scen in named_scenarios():
            dump[name] = run_case(scen, {})
    sys.stdout.write(json.dumps(dump, indent=2))


if __name__ == "__main__":
    main()
