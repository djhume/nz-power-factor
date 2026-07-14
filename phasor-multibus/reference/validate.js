/* validate.js — cross-validate powerflow.js against pandapower.
 *
 *   node reference/validate.js
 *
 * Stage A (core NR): OLTC taps neutral (t=1), DTx fixed. Compare bus |V|, angle
 *   AND branch flows (P/Q, from & to ends) to pandapower. Validates the Ybus
 *   (lines + tap transformers + shunts), the NR core, and the branch-flow
 *   expressions (S = V·conj(I) with the tap stamp).
 * Stage B (JS OLTC loop): run powerflow.js's OLTC outer loop, fix the taps it
 *   picks in pandapower, and confirm voltages+flows still match (the tap point
 *   is a true PF solution). ALSO assert each regulated bus is within its
 *   deadband (or tapped out) and res.tapConverged is true.
 * Stage C (independent tap SELECTION): pandapower's OWN DiscreteTapControl picks
 *   the taps from scratch; assert its regulated buses land in band and its
 *   selected tap steps agree with the JS controller (±1 step). This validates
 *   the tap-selection logic, not just the operating point.
 * Stage D (tap-out): force an OLTC to its limit; assert powerflow.js flags
 *   tappedOut and the escaped bus leaves the band, and cross-check the clamped
 *   operating point against pandapower.
 */
'use strict';
const fs = require('fs');
const path = require('path');
const { execFileSync } = require('child_process');

const DIR = path.resolve(__dirname, '..');
const PF = require(path.join(DIR, 'powerflow.js'));
const PARAMS = JSON.parse(fs.readFileSync(path.join(DIR, 'params.json'), 'utf8'));
const PY = '/home/dave/gridlytics/.venv/bin/python';
const REF = path.join(__dirname, 'pandapower_ref.py');

const VM_TOL = 1e-9;      // pu   (achieved floor ~5e-15; this is a regression guard)
const VA_TOL = 1e-7;      // deg
const FLOW_TOL = 1e-6;    // MW / MVAr

const SCENARIOS = Object.entries(PARAMS.scenarios)
  .filter(([k, v]) => !k.startsWith('_') && v && typeof v === 'object' && 'P' in v);

function buildNet(scenario) {
  const net = JSON.parse(JSON.stringify({
    baseMVA: PARAMS.meta.baseMVA, buses: PARAMS.buses, branches: PARAMS.branches
  }));
  for (const b of net.buses) {
    if (b.id === 'B8') { b.P = scenario.P; b.Q = scenario.Q; }
    if (b.id === 'B1') { b.Bs = scenario.deviceBs || 0; }
  }
  return net;
}
function setNeutralTaps(net) { for (const br of net.branches) if (br.oltc) br.tap = 1.0; }

function pyDump(mode, extraArg) {
  const args = [REF, mode];
  if (extraArg) args.push(extraArg);
  return JSON.parse(execFileSync(PY, args, { encoding: 'utf8', maxBuffer: 1 << 24 }));
}

function fmt(x) { return x.toExponential(2); }

// compare bus V/angle and branch flows for one case; append a row
function compare(jsRes, ppCase, label, rows) {
  let maxVm = 0, maxVa = 0, maxFl = 0;
  for (const bus of jsRes.buses) {
    const p = ppCase.buses[bus.id]; if (!p) continue;
    maxVm = Math.max(maxVm, Math.abs(bus.Vm - p.vm));
    let dVa = Math.abs(bus.VaDeg - p.va); if (dVa > 180) dVa = Math.abs(dVa - 360);
    maxVa = Math.max(maxVa, dVa);
  }
  for (const br of jsRes.branches) {
    const p = ppCase.branches[br.name]; if (!p) continue;   // br.name === branch id
    maxFl = Math.max(maxFl,
      Math.abs(br.Pfrom - p.Pfrom), Math.abs(br.Qfrom - p.Qfrom),
      Math.abs(br.Pto - p.Pto), Math.abs(br.Qto - p.Qto));
  }
  const ok = maxVm < VM_TOL && maxVa < VA_TOL && maxFl < FLOW_TOL;
  rows.push({ case: label, maxVm, maxVa, maxFl, pass: ok });
  return ok;
}
function printTable(rows) {
  console.log('case'.padEnd(14), 'max|dVm|'.padStart(11), 'max|dVa|'.padStart(11),
    'max|dFlow|'.padStart(11), ' pass');
  for (const r of rows) console.log(r.case.padEnd(14), fmt(r.maxVm).padStart(11),
    fmt(r.maxVa).padStart(11), fmt(r.maxFl).padStart(11), r.pass ? ' OK' : ' **FAIL**');
}
// branch id for a tap-report row (matched by transformer name)
function brId(name) { return PARAMS.branches.find(b => b.name === name).id; }

function main() {
  let ok = true;

  // ---- Stage A: core NR (neutral taps) ----
  console.log('=== Stage A: core Newton-Raphson (neutral OLTC taps) — V, angle, flows ===');
  const refNeutral = pyDump('neutral');
  const rowsA = [];
  for (const [name, scen] of SCENARIOS) {
    const net = buildNet(scen); setNeutralTaps(net);
    const res = PF.solveNR(net, { tol: 1e-13 });
    ok = compare(res, refNeutral[name], name, rowsA) && ok;
  }
  printTable(rowsA);

  // ---- Stage B: JS OLTC loop, pandapower confirms + deadband assertion ----
  console.log('\n=== Stage B: OLTC outer loop (JS picks taps; pandapower confirms) ===');
  const jsTaps = {}, jsRes = {};
  for (const [name, scen] of SCENARIOS) {
    const res = PF.solve(buildNet(scen), { tol: 1e-13 });
    jsRes[name] = res;
    jsTaps[name] = {};
    for (const t of res.taps) jsTaps[name][brId(t.name)] = t.steps;
  }
  const tapFile = path.join(__dirname, 'js_taps.json');
  fs.writeFileSync(tapFile, JSON.stringify(jsTaps, null, 2));
  const refTaps = pyDump('taps', tapFile);
  const rowsB = [];
  for (const [name] of SCENARIOS) ok = compare(jsRes[name], refTaps[name], name, rowsB) && ok;
  printTable(rowsB);

  // deadband + tapConverged assertions
  console.log('\n  deadband check (each regulated bus in band, or tapped out):');
  for (const [name] of SCENARIOS) {
    const res = jsRes[name];
    for (const t of res.taps) {
      const inBand = Math.abs(t.vctrl - t.vset) <= t.band + 1e-9;
      const good = inBand || t.tappedOut;
      if (!good) ok = false;
      console.log('   ', name.padEnd(13), t.name.split(' ')[0].padEnd(8),
        'Vctrl=' + t.vctrl.toFixed(4), 'set=' + t.vset.toFixed(3),
        (inBand ? 'in-band' : (t.tappedOut ? 'tapped-out:' + t.tappedOut : '**OUT OF BAND**')));
    }
    if (!res.tapConverged) { ok = false; console.log('    ', name, '**tapConverged=false**'); }
  }

  // ---- Stage C: independent tap SELECTION via pandapower's own controller ----
  console.log('\n=== Stage C: independent tap selection (pandapower DiscreteTapControl) ===');
  const refAuto = pyDump('autotap');
  console.log('  scen           tx        JS_steps  pp_steps  JS_Vctrl  pp_Vctrl  band');
  for (const [name] of SCENARIOS) {
    for (const t of jsRes[name].taps) {
      const id = brId(t.name);
      const ppStep = refAuto[name].taps[id];
      const ppVm = refAuto[name].buses[t.ctrlBus].vm;
      const ppInBand = Math.abs(ppVm - t.vset) <= t.band + 1e-9;
      const stepAgree = Math.abs(t.steps - ppStep) <= 1;
      if (!(ppInBand && stepAgree)) ok = false;
      console.log('  ', name.padEnd(13), t.name.split(' ')[0].padEnd(8),
        String(t.steps).padStart(6), String(ppStep).padStart(9),
        '  ' + t.vctrl.toFixed(4), '  ' + ppVm.toFixed(4),
        (ppInBand && stepAgree) ? '  ok' : '  **MISMATCH**');
    }
  }

  // ---- Stage D: tap-out ----
  console.log('\n=== Stage D: tap-out (OLTC hits its limit; regulated bus escapes) ===');
  const doScen = { P: 4, Q: -40, deviceBs: 0 };          // extreme leading overnight
  const doNet = buildNet(doScen);
  for (const br of doNet.branches) if (br.id === 'tx_220_33') br.oltc.tmax = 1.0125; // +1 step only
  const doRes = PF.solve(doNet, { tol: 1e-13 });
  const gxp = doRes.taps.find(t => brId(t.name) === 'tx_220_33');
  const tappedOut = !!gxp.tappedOut, escaped = Math.abs(gxp.vctrl - gxp.vset) > gxp.band;
  console.log('   tx_220_33: steps=' + gxp.steps, 'tappedOut=' + gxp.tappedOut,
    'Vctrl=' + gxp.vctrl.toFixed(4), '(target 1.030 ±0.0075) escaped=' + escaped);
  // cross-check the clamped operating point against pandapower
  const doTapFile = path.join(__dirname, 'js_taps_tapout.json');
  const doTaps = { tapout: { _scenario: doScen } };
  for (const t of doRes.taps) doTaps.tapout[brId(t.name)] = t.steps;
  fs.writeFileSync(doTapFile, JSON.stringify(doTaps, null, 2));
  const refDo = pyDump('taps', doTapFile);
  const rowsD = [];
  compare(doRes, refDo.tapout, 'tapout', rowsD);
  printTable(rowsD);
  if (!(tappedOut && escaped && rowsD[0].pass)) ok = false;

  console.log('\n' + (ok
    ? 'RESULT: PASS — powerflow.js matches pandapower on voltages, angles, flows, '
      + 'tap selection and tap-out (Vm<' + fmt(VM_TOL) + ' pu, flows<' + fmt(FLOW_TOL) + ' MVA).'
    : 'RESULT: FAIL — see tables above.'));
  process.exit(ok ? 0 : 1);
}

main();
