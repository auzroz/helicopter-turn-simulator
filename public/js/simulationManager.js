import { MAX_SIMULATIONS, getSimColor } from './utils.js';
import { SimulationInstance } from './simulation.js';

export class SimulationManager {
    constructor() {
        this.simulations = [];
        this.animationId = null;
        this.lastTimestamp = 0;
        this.onUpdate = null;   // callback(simulations) called each frame
        this.accumulators = new Map(); // per-sim time accumulators
    }

    addSimulation(presetId, conditions) {
        if (this.simulations.length >= MAX_SIMULATIONS) return null;
        const color = getSimColor(this.simulations.length);
        const sim = new SimulationInstance(presetId, conditions, color);
        sim.compute();
        this.simulations.push(sim);
        this.accumulators.set(sim.id, 0);

        if (!this.animationId) this.start();
        return sim;
    }

    removeSimulation(id) {
        this.simulations = this.simulations.filter(s => s.id !== id);
        this.accumulators.delete(id);
        if (this.simulations.length === 0) this.stop();
    }

    restartSimulation(id) {
        const sim = this.simulations.find(s => s.id === id);
        if (sim) {
            sim.restart();
            this.accumulators.set(id, 0);
            if (!this.animationId) this.start();
        }
    }

    clearAll() {
        this.stop();
        this.simulations = [];
        this.accumulators.clear();
    }

    start() {
        this.lastTimestamp = 0;
        const tick = (timestamp) => {
            if (!this.lastTimestamp) this.lastTimestamp = timestamp;
            const elapsed = (timestamp - this.lastTimestamp) / 1000; // seconds
            this.lastTimestamp = timestamp;

            // Advance each simulation by elapsed time, using per-step dt
            // so the helicopter moves faster downwind and slower upwind
            for (const sim of this.simulations) {
                if (!sim.results || !sim.isRunning) continue;
                let acc = this.accumulators.get(sim.id) + elapsed;
                while (true) {
                    const stepDt = sim.results.dt[sim.currentStep];
                    if (acc < stepDt) break;
                    acc -= stepDt;
                    sim.step();
                }
                this.accumulators.set(sim.id, acc);
            }

            if (this.onUpdate) this.onUpdate(this.simulations);
            this.animationId = requestAnimationFrame(tick);
        };
        this.animationId = requestAnimationFrame(tick);
    }

    stop() {
        if (this.animationId) {
            cancelAnimationFrame(this.animationId);
            this.animationId = null;
        }
    }
}
