import { getAllPresets, getPreset } from './helicopterPresets.js';
import { SimulationManager } from './simulationManager.js';
import { drawScene } from './rendererCanvas.js';
import { drawTorqueGauge, drawBankGauge, drawAirspeedGauge } from './rendererGauges.js';
import { MAX_SIMULATIONS } from './utils.js';

const manager = new SimulationManager();
const simCards = new Map(); // sim.id -> { card, gauges }

// DOM refs
const canvas        = document.getElementById('simulation-canvas');
const sidebar       = document.getElementById('sim-sidebar');
const presetSelect  = document.getElementById('helicopter-type');
const weightInput   = document.getElementById('gross-weight');
const headingInput  = document.getElementById('heading');
const windDirInput  = document.getElementById('wind-direction');
const windSpdInput  = document.getElementById('wind-speed');
const radiusInput   = document.getElementById('turn-radius');
const airspeedInput = document.getElementById('airspeed');
const turnDirSelect = document.getElementById('turn-direction');
const addBtn        = document.getElementById('add-simulation');
const clearBtn      = document.getElementById('clear-all');
const placeholder   = document.getElementById('sidebar-placeholder');

// Populate helicopter type dropdown
function populatePresets() {
    const presets = getAllPresets();
    for (const p of presets) {
        const opt = document.createElement('option');
        opt.value = p.id;
        opt.textContent = `${p.name}  (${p.category})`;
        presetSelect.appendChild(opt);
    }
    // Set initial weight from first preset
    updateWeightFromPreset();
}

// Update weight input when helicopter type changes
function updateWeightFromPreset() {
    const preset = getPreset(presetSelect.value);
    if (!preset) return;
    weightInput.value = preset.weight.typical;
    weightInput.min = preset.weight.empty;
    weightInput.max = preset.weight.maxGross;
}

// Read conditions from the setup panel
function readConditions() {
    return {
        heading:       parseFloat(headingInput.value) || 0,
        windDirection: parseFloat(windDirInput.value) || 0,
        windSpeed:     parseFloat(windSpdInput.value) || 0,
        turnRadius:    parseFloat(radiusInput.value) || 500,
        airspeed:      parseFloat(airspeedInput.value) || 60,
        weightLbs:     parseFloat(weightInput.value) || 16000,
        turnDirection: turnDirSelect.value
    };
}

// Create a simulation card in the sidebar
function createSimCard(sim) {
    const card = document.createElement('div');
    card.className = 'sim-card';
    card.style.borderLeftColor = sim.color;

    // Header
    const header = document.createElement('div');
    header.className = 'sim-card-header';
    const title = document.createElement('span');
    title.className = 'sim-card-title';
    title.textContent = sim.label;
    title.style.color = sim.color;
    const removeBtn = document.createElement('button');
    removeBtn.className = 'sim-remove-btn';
    removeBtn.textContent = '\u00d7';
    removeBtn.onclick = () => removeSim(sim.id);
    header.appendChild(title);
    header.appendChild(removeBtn);
    card.appendChild(header);

    // Gauges row
    const gaugeRow = document.createElement('div');
    gaugeRow.className = 'gauge-row';

    const bankCanvas = document.createElement('canvas');
    const torqueCanvas = document.createElement('canvas');
    const iasCanvas = document.createElement('canvas');

    for (const c of [bankCanvas, torqueCanvas, iasCanvas]) {
        const wrap = document.createElement('div');
        wrap.className = 'gauge-wrap';
        wrap.appendChild(c);
        gaugeRow.appendChild(wrap);
    }
    card.appendChild(gaugeRow);

    // Parameter chain display
    const chain = document.createElement('div');
    chain.className = 'param-chain';
    card.appendChild(chain);

    sidebar.appendChild(card);

    simCards.set(sim.id, { card, bankCanvas, torqueCanvas, iasCanvas, chain });
    updateSimCard(sim);
    updateChain(sim);
}

function updateSimCard(sim) {
    const entry = simCards.get(sim.id);
    if (!entry) return;
    const state = sim.getCurrentState();
    if (!state) return;

    drawBankGauge(entry.bankCanvas, state.bankAngleDeg, sim.color);
    drawTorqueGauge(entry.torqueCanvas, state.torquePercent, sim.color);
    drawAirspeedGauge(entry.iasCanvas, state.groundSpeedKnots, sim.preset.performance.vne, sim.color);
}

function updateChain(sim) {
    const entry = simCards.get(sim.id);
    if (!entry) return;
    const summary = sim.getSummary();
    if (!summary) return;

    const torqueWarn = summary.maxTorque > 100
        ? ' <span class="chain-warn">EXCEEDS MAX</span>'
        : '';

    const bankStr = summary.minBankDeg === summary.maxBankDeg
        ? `${summary.minBankDeg.toFixed(1)}\u00b0`
        : `${summary.minBankDeg.toFixed(1)}-${summary.maxBankDeg.toFixed(1)}\u00b0`;

    const gsStr = summary.minGroundSpeedKnots === summary.maxGroundSpeedKnots
        ? `${summary.minGroundSpeedKnots.toFixed(0)}kt`
        : `${summary.minGroundSpeedKnots.toFixed(0)}-${summary.maxGroundSpeedKnots.toFixed(0)}kt`;

    entry.chain.innerHTML =
        `<span class="chain-label">${summary.weightLbs.toLocaleString()}lbs</span> ` +
        `<span class="chain-arrow">\u2192</span> ` +
        `<span class="chain-label">GS</span> ${gsStr} ` +
        `<span class="chain-arrow">\u2192</span> ` +
        `<span class="chain-label">Bank</span> ${bankStr} ` +
        `<span class="chain-arrow">\u2192</span> ` +
        `<span class="chain-label">Torque</span> ${summary.minTorque.toFixed(1)}-${summary.maxTorque.toFixed(1)}%` +
        torqueWarn;
}

function removeSim(id) {
    manager.removeSimulation(id);
    const entry = simCards.get(id);
    if (entry) {
        entry.card.remove();
        simCards.delete(id);
    }
    updateAddButton();
    if (manager.simulations.length === 0) {
        // Clear canvas
        const ctx = canvas.getContext('2d');
        ctx.clearRect(0, 0, canvas.width, canvas.height);
    }
}

function updateAddButton() {
    addBtn.disabled = manager.simulations.length >= MAX_SIMULATIONS;
    placeholder.style.display = manager.simulations.length === 0 ? 'block' : 'none';
}

// Animation callback — called every frame by the manager
manager.onUpdate = (simulations) => {
    drawScene(canvas, simulations);
    for (const sim of simulations) {
        updateSimCard(sim);
    }
};

// Event handlers
presetSelect.addEventListener('change', updateWeightFromPreset);

addBtn.addEventListener('click', () => {
    const presetId = presetSelect.value;
    const conditions = readConditions();
    const sim = manager.addSimulation(presetId, conditions);
    if (sim) {
        createSimCard(sim);
        updateAddButton();
    }
});

clearBtn.addEventListener('click', () => {
    manager.clearAll();
    for (const [id, entry] of simCards) {
        entry.card.remove();
    }
    simCards.clear();
    updateAddButton();
    const ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, canvas.width, canvas.height);
});

// Handle canvas resize
window.addEventListener('resize', () => {
    // Canvas is re-setup each frame in drawScene, so nothing extra needed
});

// Init
populatePresets();
updateAddButton();
