/* ============================================================================
 * powerflow.js — self-contained AC Newton-Raphson power flow for the
 * multi-bus phasor tool (phasor-multibus).
 *
 * No DOM, no D3, no dependencies. Runs in the browser (attaches to
 * window.PowerFlow) AND under Node (module.exports) so the SAME solver that
 * draws the picture is the one validated against pandapower.
 *
 * Scope: small radial/meshed per-unit networks (< ~30 buses). Dense Jacobian,
 * polar Newton-Raphson, Gaussian elimination. Supports:
 *   - slack / PQ / PV buses
 *   - series branches (lines) with total shunt charging b (pi model)
 *   - off-nominal tap transformers (tap on the FROM / HV side, MATPOWER conv.)
 *   - bus shunt admittance Gs + jBs (cap banks, reactors, STATCOM-as-fixed-B)
 *   - an OLTC outer loop that steps discrete taps to hold a controlled bus
 *     within a deadband of its target (with tap-limit / "tapped-out" state)
 *
 * Per-unit: all impedances/admittances/powers on a common system MVA base
 * (net.baseMVA). Voltages are per-unit on each bus's own kV base — so the
 * off-nominal tap ratio carries the nominal turns ratio implicitly (a bus's
 * 1.0 pu means its own nominal kV). See PHASE1_VALIDATION.md.
 *
 * Sign convention: bus P/Q are LOADS drawn from the network (P>0 = consuming).
 * Injection = -load + shunt/gen. Q_shunt Bs>0 injects vars (capacitive).
 * ========================================================================== */
(function (root, factory) {
  if (typeof module === 'object' && module.exports) module.exports = factory();
  else root.PowerFlow = factory();
})(typeof self !== 'undefined' ? self : this, function () {
  'use strict';

  // ----- small complex helpers (series admittance only needs these) ---------
  function cInv(re, im) {            // 1 / (re + j im)
    var d = re * re + im * im;
    return [re / d, -im / d];
  }

  /* Build the bus admittance matrix Ybus (dense) as separate G, B arrays.
   * Returns {G, B, n, index} where index maps bus id -> matrix row.
   * Transformer tap `t` is the off-nominal ratio on the FROM side (HV):
   *   Yff = (y + j b/2) / t^2 ,  Yft = -y / t ,  Ytf = -y / t ,  Ytt = y + j b/2
   * For a plain line t defaults to 1 and b is the total line charging.        */
  function buildYbus(net) {
    var buses = net.buses, branches = net.branches;
    var n = buses.length;
    var index = {};
    for (var i = 0; i < n; i++) index[buses[i].id] = i;

    var G = [], B = [];
    for (i = 0; i < n; i++) { G.push(new Array(n).fill(0)); B.push(new Array(n).fill(0)); }

    for (var k = 0; k < branches.length; k++) {
      var br = branches[k];
      var f = index[br.from], tb = index[br.to];
      var r = br.r || 0, x = br.x || 0, b = br.b || 0;
      var t = (br.tap === undefined || br.tap === null) ? 1 : br.tap;
      if (r === 0 && x === 0)
        throw new Error('branch ' + br.from + '->' + br.to + ' has zero series impedance');
      if (Math.abs(t) < 1e-9)
        throw new Error('branch ' + br.from + '->' + br.to + ' has near-zero tap ratio');
      var y = cInv(r, x);                 // series admittance g + jb_series
      var yg = y[0], yb = y[1];
      var t2 = t * t;

      // Yff += (y + j b/2)/t^2
      G[f][f]  += yg / t2;
      B[f][f]  += (yb + b / 2) / t2;
      // Ytt += y + j b/2
      G[tb][tb] += yg;
      B[tb][tb] += yb + b / 2;
      // Yft = Ytf = -y / t
      G[f][tb] += -yg / t;  B[f][tb] += -yb / t;
      G[tb][f] += -yg / t;  B[tb][f] += -yb / t;
    }

    // Bus fixed shunts (Gs + jBs), in pu on baseMVA
    for (i = 0; i < n; i++) {
      G[i][i] += buses[i].Gs || 0;
      B[i][i] += buses[i].Bs || 0;
    }
    return { G: G, B: B, n: n, index: index };
  }

  // Dense linear solve A x = rhs via Gaussian elimination w/ partial pivoting.
  function linsolve(A, rhs) {
    var n = rhs.length;
    // augment
    var M = [];
    for (var i = 0; i < n; i++) M.push(A[i].slice().concat(rhs[i]));
    for (var col = 0; col < n; col++) {
      // pivot
      var piv = col, best = Math.abs(M[col][col]);
      for (var rr = col + 1; rr < n; rr++) {
        var v = Math.abs(M[rr][col]);
        if (v > best) { best = v; piv = rr; }
      }
      if (best < 1e-14) throw new Error('singular Jacobian at col ' + col);
      if (piv !== col) { var tmp = M[piv]; M[piv] = M[col]; M[col] = tmp; }
      var d = M[col][col];
      for (var cc = col; cc <= n; cc++) M[col][cc] /= d;
      for (rr = 0; rr < n; rr++) {
        if (rr === col) continue;
        var factor = M[rr][col];
        if (factor === 0) continue;
        for (cc = col; cc <= n; cc++) M[rr][cc] -= factor * M[col][cc];
      }
    }
    var x = new Array(n);
    for (i = 0; i < n; i++) x[i] = M[i][n];
    return x;
  }

  /* Core Newton-Raphson AC power flow (polar). Taps are whatever is currently
   * on each branch (the OLTC outer loop sets them). Returns per-bus V, theta
   * and per-branch complex flows. Throws if it fails to converge. */
  function solveNR(net, opt) {
    opt = opt || {};
    var tol = opt.tol || 1e-9, maxit = opt.maxit || 40;
    var Y = buildYbus(net);
    var G = Y.G, B = Y.B, n = Y.n;
    var buses = net.buses;

    // bus types
    var SLACK = 3, PV = 2, PQ = 1;
    var type = new Array(n);
    var V = new Array(n), th = new Array(n);
    var Psp = new Array(n), Qsp = new Array(n);   // specified NET injection (pu)
    for (var i = 0; i < n; i++) {
      var bs = buses[i];
      type[i] = bs.type === 'slack' ? SLACK : (bs.type === 'PV' ? PV : PQ);
      V[i]  = bs.Vm !== undefined ? bs.Vm : (bs.type === 'slack' || bs.type === 'PV' ? (bs.Vset || 1.0) : 1.0);
      th[i] = bs.Va !== undefined ? bs.Va : 0;
      // net injection = generation - load. We carry load as P/Q (drawn) and
      // optional Pg/Qg (injected). PV bus uses Pg as specified P injection.
      var Pload = bs.P || 0, Qload = bs.Q || 0;
      var Pg = bs.Pg || 0, Qg = bs.Qg || 0;
      Psp[i] = (Pg - Pload) / net.baseMVA;
      Qsp[i] = (Qg - Qload) / net.baseMVA;
    }

    // exactly one slack (reference) bus required
    var nslack = 0;
    for (i = 0; i < n; i++) if (type[i] === SLACK) nslack++;
    if (nslack !== 1)
      throw new Error('power flow needs exactly one slack bus, found ' + nslack);

    // index sets for unknowns
    var pvpq = [], pq = [];
    for (i = 0; i < n; i++) {
      if (type[i] !== SLACK) pvpq.push(i);
      if (type[i] === PQ) pq.push(i);
    }
    var npv = pvpq.length, npq = pq.length;

    function calcPQ() {
      var P = new Array(n).fill(0), Q = new Array(n).fill(0);
      for (var a = 0; a < n; a++) {
        for (var b = 0; b < n; b++) {
          var dth = th[a] - th[b];
          var c = Math.cos(dth), s = Math.sin(dth);
          P[a] += V[a] * V[b] * (G[a][b] * c + B[a][b] * s);
          Q[a] += V[a] * V[b] * (G[a][b] * s - B[a][b] * c);
        }
      }
      return { P: P, Q: Q };
    }

    var it = 0, converged = false;
    for (; it < maxit; it++) {
      var pqc = calcPQ();
      // mismatch
      var dP = new Array(npv), dQ = new Array(npq);
      var maxmis = 0, ii, bad = false;
      for (ii = 0; ii < npv; ii++) {
        dP[ii] = Psp[pvpq[ii]] - pqc.P[pvpq[ii]];
        if (!isFinite(dP[ii])) bad = true;
        if (Math.abs(dP[ii]) > maxmis) maxmis = Math.abs(dP[ii]);
      }
      for (ii = 0; ii < npq; ii++) {
        dQ[ii] = Qsp[pq[ii]] - pqc.Q[pq[ii]];
        if (!isFinite(dQ[ii])) bad = true;
        if (Math.abs(dQ[ii]) > maxmis) maxmis = Math.abs(dQ[ii]);
      }
      // Math.abs(NaN) > maxmis is always false, so a NaN would otherwise leave
      // maxmis=0 and falsely "converge" — catch it explicitly.
      if (bad) throw new Error('non-finite power mismatch (check for zero-impedance branch, tap~0, or bad input)');
      if (maxmis < tol) { converged = true; break; }

      // Build Jacobian [ H N ; J L ]; size (npv+npq)
      var m = npv + npq;
      var Jac = [];
      for (ii = 0; ii < m; ii++) Jac.push(new Array(m).fill(0));
      var rhs = new Array(m);
      for (ii = 0; ii < npv; ii++) rhs[ii] = dP[ii];
      for (ii = 0; ii < npq; ii++) rhs[npv + ii] = dQ[ii];

      // Partial derivatives. Use standard formulas.
      // dP_i/dth_j, dP_i/dV_j, dQ_i/dth_j, dQ_i/dV_j
      function Pi(i) { return pqc.P[i]; }
      function Qi(i) { return pqc.Q[i]; }
      var a, b2;
      for (var r1 = 0; r1 < npv; r1++) {
        a = pvpq[r1];
        // H: dP/dtheta  (cols over pvpq)
        for (var c1 = 0; c1 < npv; c1++) {
          b2 = pvpq[c1];
          if (a === b2) {
            Jac[r1][c1] = -Qi(a) - B[a][a] * V[a] * V[a];
          } else {
            var dth1 = th[a] - th[b2];
            Jac[r1][c1] = V[a] * V[b2] * (G[a][b2] * Math.sin(dth1) - B[a][b2] * Math.cos(dth1));
          }
        }
        // N: dP/dV  (cols over pq)
        for (c1 = 0; c1 < npq; c1++) {
          b2 = pq[c1];
          if (a === b2) {
            Jac[r1][npv + c1] = Pi(a) / V[a] + G[a][a] * V[a];
          } else {
            var dth2 = th[a] - th[b2];
            Jac[r1][npv + c1] = V[a] * (G[a][b2] * Math.cos(dth2) + B[a][b2] * Math.sin(dth2));
          }
        }
      }
      for (var r2 = 0; r2 < npq; r2++) {
        a = pq[r2];
        // J: dQ/dtheta (cols over pvpq)
        for (c1 = 0; c1 < npv; c1++) {
          b2 = pvpq[c1];
          if (a === b2) {
            Jac[npv + r2][c1] = Pi(a) - G[a][a] * V[a] * V[a];
          } else {
            var dth3 = th[a] - th[b2];
            Jac[npv + r2][c1] = -V[a] * V[b2] * (G[a][b2] * Math.cos(dth3) + B[a][b2] * Math.sin(dth3));
          }
        }
        // L: dQ/dV (cols over pq)
        for (c1 = 0; c1 < npq; c1++) {
          b2 = pq[c1];
          if (a === b2) {
            Jac[npv + r2][npv + c1] = Qi(a) / V[a] - B[a][a] * V[a];
          } else {
            var dth4 = th[a] - th[b2];
            Jac[npv + r2][npv + c1] = V[a] * (G[a][b2] * Math.sin(dth4) - B[a][b2] * Math.cos(dth4));
          }
        }
      }

      var dx = linsolve(Jac, rhs);
      for (ii = 0; ii < npv; ii++) th[pvpq[ii]] += dx[ii];
      for (ii = 0; ii < npq; ii++) V[pq[ii]] += dx[npv + ii];
    }

    if (!converged) throw new Error('NR did not converge (maxmis after ' + it + ' its)');

    // final injections
    var fin = calcPQ();
    var out = { converged: converged, iterations: it, index: Y.index, buses: [] };
    for (i = 0; i < n; i++) {
      out.buses.push({
        id: buses[i].id, name: buses[i].name, kV: buses[i].kV,
        Vm: V[i], Va: th[i], VaDeg: th[i] * 180 / Math.PI,
        Pinj: fin.P[i] * net.baseMVA, Qinj: fin.Q[i] * net.baseMVA
      });
    }

    // branch flows (from->to), pu then MVA
    out.branches = [];
    for (var k = 0; k < net.branches.length; k++) {
      var br = net.branches[k];
      var f = Y.index[br.from], tt = Y.index[br.to];
      var rr = br.r || 0, xx = br.x || 0, bb = br.b || 0;
      var tp = (br.tap == null) ? 1 : br.tap;
      var yy = cInv(rr, xx);
      // complex voltages
      var Vf = [V[f] * Math.cos(th[f]), V[f] * Math.sin(th[f])];
      var Vt = [V[tt] * Math.cos(th[tt]), V[tt] * Math.sin(th[tt])];
      // I_from = Yff*Vf + Yft*Vt with Yff=(y+jb/2)/t^2, Yft=-y/t
      var Yff = [yy[0] / (tp * tp), (yy[1] + bb / 2) / (tp * tp)];
      var Yft = [-yy[0] / tp, -yy[1] / tp];
      var Ytt = [yy[0], yy[1] + bb / 2];
      var Ytf = [-yy[0] / tp, -yy[1] / tp];
      var If = cAddMulSum(Yff, Vf, Yft, Vt);
      var It = cAddMulSum(Ytf, Vf, Ytt, Vt);
      // S_from = Vf * conj(If)
      var Sf = cMul(Vf, [If[0], -If[1]]);
      var St = cMul(Vt, [It[0], -It[1]]);
      out.branches.push({
        from: br.from, to: br.to, name: br.name || (br.from + '->' + br.to),
        tap: tp,
        Pfrom: Sf[0] * net.baseMVA, Qfrom: Sf[1] * net.baseMVA,
        Pto: St[0] * net.baseMVA, Qto: St[1] * net.baseMVA
      });
    }
    return out;
  }

  function cMul(a, b) { return [a[0] * b[0] - a[1] * b[1], a[0] * b[1] + a[1] * b[0]]; }
  function cAddMulSum(y1, v1, y2, v2) {
    var a = cMul(y1, v1), b = cMul(y2, v2);
    return [a[0] + b[0], a[1] + b[1]];
  }

  /* OLTC outer loop.
   * Each transformer branch may carry an `oltc` spec:
   *   { ctrlBus, vset, band, tmin, tmax, step }
   * where tap is on the FROM/HV side; raising tap LOWERS the controlled LV bus.
   * We solve NR, then for each OLTC nudge its tap one step toward target if the
   * controlled bus is outside the deadband and taps remain in range; repeat
   * until all OLTCs are settled or a max iteration count is hit. A transformer
   * that wants to move further but is at tmin/tmax is flagged tappedOut.
   * Returns the final NR solution with .taps[] and .tappedOut[] annotations. */
  function solve(net, opt) {
    opt = opt || {};
    var maxTapIters = opt.maxTapIters || 60;

    // Work on a deep copy so repeated solve() calls on the same net are
    // idempotent — an interactive tool calls this every slider move, and the
    // OLTC search is a path-dependent stepping loop, so it must always start
    // from the net's declared taps, never from a previous call's result.
    var work = JSON.parse(JSON.stringify(net));
    var res = null;
    var idx = {};
    for (var i = 0; i < work.buses.length; i++) idx[work.buses[i].id] = i;

    var oltcBr = [];
    for (var k = 0; k < work.branches.length; k++) {
      if (work.branches[k].oltc) oltcBr.push(work.branches[k]);
    }

    // Iterate: solve, then step each OLTC one tap toward its target. Terminate
    // when nothing moves (converged / all pinned at a limit). A genuine
    // limit-cycle (deadband < one step) revisits an entire tap vector — detect
    // that by remembering visited tap vectors, so a normal single-step
    // overshoot-and-correct is NOT mistaken for hunting.
    var seen = {};
    var eps = 1e-9;
    for (var pass = 0; pass < maxTapIters; pass++) {
      res = solveNR(work, opt);
      var moved = false;
      for (var j = 0; j < oltcBr.length; j++) {
        var br = oltcBr[j], o = br.oltc;
        var err = res.buses[idx[o.ctrlBus]].Vm - o.vset;
        if (Math.abs(err) <= o.band) { br._tappedOut = null; continue; }
        // vm too high -> raise HV tap to pull LV down; vm low -> lower tap
        var newTap = br.tap + (err > 0 ? +1 : -1) * o.step;
        if (newTap > o.tmax + eps) {
          if (br.tap < o.tmax - eps) { br.tap = o.tmax; moved = true; }
          br._tappedOut = 'max';
        } else if (newTap < o.tmin - eps) {
          if (br.tap > o.tmin + eps) { br.tap = o.tmin; moved = true; }
          br._tappedOut = 'min';
        } else { br.tap = newTap; br._tappedOut = null; moved = true; }
      }
      if (!moved) break;                                   // settled
      var key = oltcBr.map(function (b) { return b.tap.toFixed(6); }).join(',');
      if (seen[key]) break;                                // limit cycle -> stop
      seen[key] = 1;
    }

    // Report + convergence: a changer that ends neither in band nor tapped out
    // is hunting (deadband < step), which means the loop could not settle it.
    var tapConverged = true;
    res.taps = [];
    for (k = 0; k < work.branches.length; k++) {
      var b = work.branches[k];
      if (!b.oltc) continue;
      var vctrl = res.buses[idx[b.oltc.ctrlBus]].Vm;
      var inBand = Math.abs(vctrl - b.oltc.vset) <= b.oltc.band + eps;
      var hunting = !inBand && !b._tappedOut;
      if (hunting) tapConverged = false;
      res.taps.push({
        name: b.name || (b.from + '->' + b.to), tap: b.tap,
        steps: Math.round((b.tap - 1.0) / b.oltc.step),
        ctrlBus: b.oltc.ctrlBus, vset: b.oltc.vset, band: b.oltc.band, vctrl: vctrl,
        inBand: inBand, tappedOut: b._tappedOut || null, hunting: hunting
      });
    }
    res.tapConverged = tapConverged;
    return res;
  }

  return {
    buildYbus: buildYbus,
    solveNR: solveNR,
    solve: solve,
    linsolve: linsolve
  };
});
