// Renders a live physics breakdown panel showing formulas and computed values.

const HP_PER_WATT = 1 / 745.7;

function fmt(val, decimals = 1) {
    return val.toLocaleString(undefined, { minimumFractionDigits: decimals, maximumFractionDigits: decimals });
}

function fmtHP(watts) {
    return fmt(watts * HP_PER_WATT, 0) + ' hp';
}

function fmtW(watts) {
    if (watts > 1e6) return fmt(watts / 1e6, 1) + ' MW';
    if (watts > 1e3) return fmt(watts / 1e3, 1) + ' kW';
    return fmt(watts, 0) + ' W';
}

// Build the static DOM skeleton for one simulation's explainer block.
// Returns an object with span refs for each dynamic value.
function buildExplainerBlock(container, sim) {
    const block = document.createElement('div');
    block.className = 'explainer-block';
    block.style.borderLeftColor = sim.color;

    const title = document.createElement('div');
    title.className = 'explainer-block-title';
    title.style.color = sim.color;
    title.textContent = sim.label;
    block.appendChild(title);

    const spans = {};

    function addRow(id, label, formulaHTML) {
        const row = document.createElement('div');
        row.className = 'explainer-row';

        const lbl = document.createElement('div');
        lbl.className = 'explainer-label';
        lbl.textContent = label;

        const formula = document.createElement('div');
        formula.className = 'explainer-formula';
        formula.innerHTML = formulaHTML;

        const result = document.createElement('span');
        result.className = 'explainer-result';
        spans[id] = result;

        const valDiv = document.createElement('div');
        valDiv.className = 'explainer-value';
        valDiv.appendChild(result);

        row.appendChild(lbl);
        row.appendChild(formula);
        row.appendChild(valDiv);
        block.appendChild(row);
    }

    // Section: Flight Condition
    addRow('weight', 'Weight', 'W = gross weight &times; 4.449');
    addRow('bankAngle', 'Bank Angle', '&phi; = atan(V<sub>g</sub>&sup2; / (g &middot; r))');
    addRow('loadFactor', 'Load Factor', 'n = 1 / cos(&phi;)');
    addRow('thrust', 'Thrust Required', 'T = W &middot; n');

    // Section: Induced Power
    addRow('vi0', 'Hover Induced Velocity', 'v<sub>i0</sub> = &radic;(T / (2&rho;A))');
    addRow('vi', 'Fwd-Flight Induced Vel.', 'v<sub>i</sub> = v<sub>i0</sub> / &radic;(1 + (V/v<sub>i0</sub>)&sup2;)');
    addRow('pInduced', 'Induced Power', 'P<sub>i</sub> = k &middot; T &middot; v<sub>i</sub>');

    // Section: Profile Power
    addRow('mu', 'Advance Ratio', '&mu; = V / V<sub>tip</sub>');
    addRow('pProfile', 'Profile Power', 'P<sub>p</sub> = P<sub>p0</sub>(1 + 4.65&mu;&sup2;)');

    // Section: Parasite Power
    addRow('pParasite', 'Parasite Power', 'P<sub>par</sub> = &frac12;&rho;V&sup3;S<sub>eq</sub>');

    // Section: Main Rotor
    addRow('pMainBase', 'Main Rotor (base)', 'P<sub>MR</sub> = P<sub>i</sub> + P<sub>p</sub> + P<sub>par</sub>');
    addRow('fwdCorr', 'Fwd-Flight Correction', 'factor = 1 + K&middot;&mu;&middot;(1 &minus; &mu;/&mu;<sub>max</sub>)');
    addRow('pMain', 'Main Rotor (corrected)', 'P<sub>MR</sub> &times; correction');

    // Section: Total
    addRow('pTail', 'Tail Rotor', 'P<sub>TR</sub> = 10% &times; P<sub>MR</sub>');
    addRow('pAcc', 'Accessories', 'P<sub>acc</sub> = 3% &times; P<sub>max</sub>');
    addRow('pTotal', 'Total Engine Power', '(P<sub>MR</sub> + P<sub>TR</sub> + P<sub>acc</sub>) / 0.975');
    addRow('torque', 'Torque', 'Q% = P<sub>total</sub> / P<sub>max</sub> &times; 100');

    container.appendChild(block);
    return { block, spans, simId: sim.id };
}

// Update dynamic values in an existing explainer block.
function updateBlock(refs, sim) {
    const state = sim.getCurrentState();
    if (!state) return;

    const p = sim.preset;
    const W_N = (sim.conditions.weightLbs || p.weight.typical) * 4.44822;
    const bankRad = state.bankAngleDeg * Math.PI / 180;
    const n = state.loadFactor;
    const T = W_N * n;
    const R = p.rotor.radius;
    const A = Math.PI * R * R;
    const omega = p.rotor.omega;
    const vtip = omega * R;
    const IAS_ms = sim.results.airspeedKnots * 0.514444;
    const mu = IAS_ms / vtip;

    const vi0 = Math.sqrt(T / (2 * 1.225 * A));
    const vi = vi0 / Math.sqrt(1 + (IAS_ms / vi0) ** 2);

    const pwr = state.powerComponents;
    const fwdK = 4.0;
    const vne_ms = p.performance.vne * 0.514444;
    const muMax = vne_ms / vtip;
    const fwdFactor = 1 + fwdK * mu * Math.max(0, 1 - mu / muMax);

    const s = refs.spans;
    s.weight.textContent = `= ${fmt(W_N, 0)} N  (${fmt(W_N / 4.44822, 0)} lbs)`;
    s.bankAngle.textContent = `= ${fmt(state.bankAngleDeg)}\u00b0`;
    s.loadFactor.textContent = `= ${fmt(n, 3)}`;
    s.thrust.textContent = `= ${fmt(T, 0)} N  (${fmt(T / 4.44822, 0)} lbs)`;
    s.vi0.textContent = `= ${fmt(vi0, 2)} m/s  (A = ${fmt(A, 1)} m\u00b2)`;
    s.vi.textContent = `= ${fmt(vi, 2)} m/s  (V = ${fmt(IAS_ms, 1)} m/s)`;
    s.pInduced.textContent = `= ${fmtHP(pwr.induced)}  (${fmtW(pwr.induced)})`;
    s.mu.textContent = `= ${fmt(mu, 3)}  (V<sub>tip</sub> = ${fmt(vtip, 0)} m/s)`;
    s.pProfile.textContent = `= ${fmtHP(pwr.profile)}  (${fmtW(pwr.profile)})`;
    s.pParasite.textContent = `= ${fmtHP(pwr.parasite)}  (S<sub>eq</sub> = ${fmt(p.airframe.flatPlateArea, 2)} m\u00b2)`;
    s.pMainBase.textContent = `= ${fmtHP(pwr.induced + pwr.profile + pwr.parasite)}`;
    s.fwdCorr.textContent = `= ${fmt(fwdFactor, 3)}  (+${fmt((fwdFactor - 1) * 100, 1)}%)`;
    s.pMain.textContent = `= ${fmtHP(pwr.mainRotor)}  (${fmtW(pwr.mainRotor)})`;
    s.pTail.textContent = `= ${fmtHP(pwr.tailRotor)}`;
    s.pAcc.textContent = `= ${fmtHP(pwr.accessories)}`;
    s.pTotal.textContent = `= ${fmtHP(pwr.total)}  (${fmtW(pwr.total)})`;

    const torqueColor = state.torquePercent > 100 ? '#e74c3c' : state.torquePercent > 85 ? '#f1c40f' : '#00ff88';
    s.torque.textContent = `= ${fmt(state.torquePercent)}%`;
    s.torque.style.color = torqueColor;
}

// Public API: manage explainer blocks for all active simulations.
const blockRefs = new Map();

export function updateExplainerPanel(container, simulations) {
    // Remove blocks for simulations that no longer exist
    for (const [simId, refs] of blockRefs) {
        if (!simulations.find(s => s.id === simId)) {
            refs.block.remove();
            blockRefs.delete(simId);
        }
    }

    // Add blocks for new simulations
    for (const sim of simulations) {
        if (!blockRefs.has(sim.id)) {
            const refs = buildExplainerBlock(container, sim);
            blockRefs.set(sim.id, refs);
        }
    }

    // Update all blocks
    for (const sim of simulations) {
        const refs = blockRefs.get(sim.id);
        if (refs) updateBlock(refs, sim);
    }
}

export function clearExplainerPanel() {
    for (const [, refs] of blockRefs) {
        refs.block.remove();
    }
    blockRefs.clear();
}
