import { getPreset } from './helicopterPresets.js';
import { runFullOrbit } from './physics.js';
import { msToKnots, radiansToDegrees } from './utils.js';

let nextId = 1;

export class SimulationInstance {
    constructor(presetId, conditions, color) {
        this.id = nextId++;
        this.preset = getPreset(presetId);
        this.conditions = { ...conditions };
        this.results = null;
        this.currentStep = 0;
        this.color = color;
        this.isRunning = false;
    }

    compute() {
        this.results = runFullOrbit({
            weightLbs: this.conditions.weightLbs || this.preset.weight.typical,
            headingDeg: this.conditions.heading,
            windDirectionDeg: this.conditions.windDirection,
            windSpeedKnots: this.conditions.windSpeed,
            turnRadiusM: this.conditions.turnRadius,
            airspeedKnots: this.conditions.airspeed,
            turnDirection: this.conditions.turnDirection || 'left',
            rotor: this.preset.rotor,
            airframe: this.preset.airframe,
            performance: this.preset.performance
        });
        this.currentStep = 0;
        this.isRunning = true;
    }

    step() {
        if (!this.results) return null;
        this.currentStep = (this.currentStep + 1) % this.results.numSteps;
        return this.getCurrentState();
    }

    getCurrentState() {
        if (!this.results) return null;
        const i = this.currentStep;
        return {
            x: this.results.x[i],
            y: this.results.y[i],
            groundTrackHeading: this.results.groundTrackHeading[i],
            psi: this.results.psi[i],
            bankAngleDeg: radiansToDegrees(this.results.bankAngle[i]),
            groundSpeedKnots: msToKnots(this.results.groundSpeed[i]),
            airspeedKnots: this.results.airspeedKnots,
            torquePercent: this.results.torque[i],
            powerTotal: this.results.power[i].total,
            powerComponents: this.results.power[i],
            loadFactor: this.results.loadFactors[i]
        };
    }

    getSummary() {
        if (!this.results) return null;
        return {
            minBankDeg: this.results.minBankDeg,
            maxBankDeg: this.results.maxBankDeg,
            minGroundSpeedKnots: this.results.minGroundSpeedKnots,
            maxGroundSpeedKnots: this.results.maxGroundSpeedKnots,
            minTorque: this.results.minTorque,
            maxTorque: this.results.maxTorque,
            airspeedKnots: this.conditions.airspeed,
            weightLbs: this.conditions.weightLbs || this.preset.weight.typical,
            turnRadiusM: this.conditions.turnRadius,
            windSpeedKnots: this.conditions.windSpeed
        };
    }

    get label() {
        const wt = this.conditions.weightLbs || this.preset.weight.typical;
        const wind = this.conditions.windSpeed > 0
            ? `, ${this.conditions.windSpeed}kt wind`
            : '';
        return `${this.preset.name} ${wt.toLocaleString()}lbs${wind}`;
    }
}
