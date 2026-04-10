import { lbsToNewtons, knotsToMs, degreesToRadians, kWToWatts } from './utils.js';

const RHO = 1.225;   // air density kg/m^3  (sea level ISA)
const G   = 9.81;    // gravitational acceleration m/s^2

// Compute wind vector components (m/s).
// windDirection is the heading the wind is coming FROM (degrees true).
export function windComponents(windSpeedKnots, windDirectionDeg) {
    const Vw = knotsToMs(windSpeedKnots);
    // Wind coming FROM a direction means it blows TOWARD the opposite direction.
    const dirRad = degreesToRadians(windDirectionDeg);
    return {
        x: -Vw * Math.sin(dirRad),
        y: -Vw * Math.cos(dirRad)
    };
}

// Airspeed vector at a given position angle (psi) around the orbit.
// Returns { vx, vy, magnitude }.
export function airspeedAtPosition(groundSpeedMs, psi, wind) {
    // Ground velocity components (aircraft flying a circle, psi=0 is north/top)
    const vgx = groundSpeedMs * Math.sin(psi);
    const vgy = groundSpeedMs * Math.cos(psi);

    // Airspeed = ground speed - wind (vector subtraction)
    const vax = vgx - wind.x;
    const vay = vgy - wind.y;

    return {
        x: vax,
        y: vay,
        magnitude: Math.sqrt(vax * vax + vay * vay),
        heading: Math.atan2(vax, vay)
    };
}

// Bank angle required for a given ground speed and turn radius.
export function bankAngleForTurn(groundSpeedMs, turnRadiusM) {
    return Math.atan((groundSpeedMs * groundSpeedMs) / (G * turnRadiusM));
}

// Load factor from bank angle.
export function loadFactor(bankAngleRad) {
    return 1 / Math.cos(bankAngleRad);
}

// Hover induced velocity (m/s).
export function hoverInducedVelocity(weightN, rotorRadius) {
    const A = Math.PI * rotorRadius * rotorRadius;
    return Math.sqrt(weightN / (2 * RHO * A));
}

// Forward-flight induced velocity using Glauert approximation.
export function forwardInducedVelocity(vi0, airspeedMs) {
    return vi0 / Math.sqrt(1 + (airspeedMs / vi0) ** 2);
}

// Power components (watts) at a single point in the orbit.
export function powerComponents(params) {
    const { weightN, airspeedMs, bankAngleRad, rotor, airframe } = params;
    const { radius, omega, solidity } = rotor;
    const { bladeProfileDragCoeff: Cd0, inducedPowerFactor: kFactor, flatPlateArea: Seq } = airframe;

    const n = loadFactor(bankAngleRad);
    const vi0 = hoverInducedVelocity(weightN, radius);
    const vi = forwardInducedVelocity(vi0, airspeedMs);

    const induced    = kFactor * weightN * vi;
    const profile    = (solidity * Cd0 * RHO * Math.pow(omega, 3) * Math.pow(radius, 3)) / 8;
    const parasite   = 0.5 * RHO * Math.pow(airspeedMs, 3) * Seq;
    const maneuvering = (n - 1) * induced;

    const total = induced + profile + parasite + maneuvering;

    return { induced, profile, parasite, maneuvering, total };
}

// Convert total power (watts) to torque percentage.
// torque = power / omega, then percent of max available.
export function torquePercent(powerWatts, maxContinuousPowerKW, rotorOmega) {
    const maxTorque = kWToWatts(maxContinuousPowerKW) / rotorOmega;
    const currentTorque = powerWatts / rotorOmega;
    return (currentTorque / maxTorque) * 100;
}

// Run a full 360-degree orbit simulation. Returns arrays of per-step data.
export function runFullOrbit(config) {
    const {
        weightLbs,
        headingDeg,
        windDirectionDeg,
        windSpeedKnots,
        turnRadiusM,
        groundSpeedKnots,
        rotor,
        airframe,
        performance
    } = config;

    const W  = lbsToNewtons(weightLbs);
    const Vg = knotsToMs(groundSpeedKnots);
    const wind = windComponents(windSpeedKnots, windDirectionDeg);
    const entryRad = degreesToRadians(headingDeg);

    const omega = Vg / turnRadiusM;          // turn rate (rad/s)
    const totalTime = (2 * Math.PI) / omega; // time for full orbit
    const numSteps = 360;
    const dt = totalTime / numSteps;

    const results = {
        numSteps,
        dt,
        totalTime,
        turnRate: omega,
        x: [],
        y: [],
        psi: [],
        airspeed: [],
        airspeedHeading: [],
        bankAngle: [],
        loadFactors: [],
        power: [],
        torque: [],
        groundSpeedMs: Vg,
        turnRadiusM,
        wind
    };

    const bank = bankAngleForTurn(Vg, turnRadiusM);

    for (let i = 0; i < numSteps; i++) {
        const t = i * dt;
        const psi = entryRad + omega * t;

        // Position on ground circle
        results.x.push(turnRadiusM * Math.sin(psi));
        results.y.push(turnRadiusM * Math.cos(psi));
        results.psi.push(psi);

        // Airspeed at this position
        const air = airspeedAtPosition(Vg, psi, wind);
        results.airspeed.push(air.magnitude);
        results.airspeedHeading.push(air.heading);

        // Bank angle (constant for now — see note in plan about variable bank)
        results.bankAngle.push(bank);
        results.loadFactors.push(loadFactor(bank));

        // Power
        const pwr = powerComponents({
            weightN: W,
            airspeedMs: air.magnitude,
            bankAngleRad: bank,
            rotor,
            airframe
        });
        results.power.push(pwr);

        // Torque %
        results.torque.push(
            torquePercent(pwr.total, performance.maxContinuousPower, rotor.omega)
        );
    }

    // Summary stats
    results.minAirspeedKnots = Math.min(...results.airspeed) / 0.514444;
    results.maxAirspeedKnots = Math.max(...results.airspeed) / 0.514444;
    results.minTorque = Math.min(...results.torque);
    results.maxTorque = Math.max(...results.torque);

    return results;
}
