import { getPreset } from './helicopterPresets.js';
import { runFullOrbit, runApproachPattern } from './physics.js';
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
        const cfg = {
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
        };

        if (this.conditions.approachEnabled) {
            this.results = runApproachPattern({
                ...cfg,
                approachHeadingDeg: this.conditions.approachHeading || 180,
                approachDistanceM: this.conditions.approachDistance || 1000
            });
        } else {
            this.results = runFullOrbit(cfg);
        }

        this.currentStep = 0;
        this.isRunning = true;
    }

    step() {
        if (!this.results) return null;
        if (this.results.isApproach) {
            // Don't loop — stop at end of approach
            if (this.currentStep < this.results.numSteps - 1) {
                this.currentStep++;
            } else {
                this.isRunning = false;
            }
        } else {
            this.currentStep = (this.currentStep + 1) % this.results.numSteps;
        }
        return this.getCurrentState();
    }

    restart() {
        this.currentStep = 0;
        this.isRunning = true;
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
            airspeedKnots: this.results.ias ? this.results.ias[i] : this.results.airspeedKnots,
            torquePercent: this.results.torque[i],
            powerTotal: this.results.power[i].total,
            powerComponents: this.results.power[i],
            loadFactor: this.results.loadFactors[i],
            phase: this.results.phase ? this.results.phase[i] : 'orbit',
            radius: this.results.radius ? this.results.radius[i] : this.results.turnRadiusM
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
            windSpeedKnots: this.conditions.windSpeed,
            isApproach: !!this.results.isApproach
        };
    }

    get label() {
        const wt = this.conditions.weightLbs || this.preset.weight.typical;
        const wind = this.conditions.windSpeed > 0
            ? `, ${this.conditions.windSpeed}kt wind`
            : '';
        const approach = this.conditions.approachEnabled ? ' [Approach]' : '';
        return `${this.preset.name} ${wt.toLocaleString()}lbs${wind}${approach}`;
    }
}
