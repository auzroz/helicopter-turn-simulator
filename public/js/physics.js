import { lbsToNewtons, knotsToMs, msToKnots, degreesToRadians } from './utils.js';

const RHO = 1.225;   // air density kg/m^3  (sea level ISA)
const G   = 9.81;    // gravitational acceleration m/s^2

// Compute wind vector components (m/s).
// windDirection is the heading the wind is coming FROM (degrees true).
export function windComponents(windSpeedKnots, windDirectionDeg) {
    const Vw = knotsToMs(windSpeedKnots);
    const dirRad = degreesToRadians(windDirectionDeg);
    return {
        x: -Vw * Math.sin(dirRad),
        y: -Vw * Math.cos(dirRad),
        magnitude: Vw
    };
}

// Solve for ground speed at azimuth psi given constant IAS and wind.
// The aircraft flies a circular ground track; ground velocity is tangent to circle.
// Returns ground speed in m/s (positive), or 0 if IAS < crosswind component.
export function groundSpeedFromIAS(iasMps, wind, psi) {
    // Ground track tangent direction (CCW / left turn)
    const tx = Math.cos(psi);
    const ty = -Math.sin(psi);

    // Component of wind along the track direction
    const h = tx * wind.x + ty * wind.y;

    // Quadratic: Vg² - 2·h·Vg + (|wind|² - IAS²) = 0
    const discriminant = iasMps * iasMps - wind.magnitude * wind.magnitude + h * h;
    if (discriminant < 0) return Math.max(0, h); // IAS too low to maintain track
    return h + Math.sqrt(discriminant);
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
export function hoverInducedVelocity(thrustN, rotorRadius) {
    const A = Math.PI * rotorRadius * rotorRadius;
    return Math.sqrt(thrustN / (2 * RHO * A));
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
    const thrust = weightN * n;

    // Induced velocity accounts for increased thrust in the turn
    const vi0 = hoverInducedVelocity(thrust, radius);
    const vi = forwardInducedVelocity(vi0, airspeedMs);

    // Induced power: k * T * vi
    const induced = kFactor * thrust * vi;

    // Profile power: (sigma * Cd0 / 8) * rho * A * Vtip³
    const A = Math.PI * radius * radius;
    const vtip = omega * radius;
    const profile = (solidity * Cd0 / 8) * RHO * A * Math.pow(vtip, 3);

    // Parasite power
    const parasite = 0.5 * RHO * Math.pow(airspeedMs, 3) * Seq;

    const total = induced + profile + parasite;

    return { induced, profile, parasite, total };
}

// Convert total power (watts) to torque percentage.
export function torquePercent(powerWatts, maxContinuousPowerKW, rotorOmega) {
    const maxPowerW = maxContinuousPowerKW * 1000;
    const maxTorque = maxPowerW / rotorOmega;
    const currentTorque = powerWatts / rotorOmega;
    return (currentTorque / maxTorque) * 100;
}

// Run a full 360-degree orbit simulation.
// IAS is constant; ground speed, bank angle, and power vary with wind.
export function runFullOrbit(config) {
    const {
        weightLbs,
        headingDeg,
        windDirectionDeg,
        windSpeedKnots,
        turnRadiusM,
        airspeedKnots,
        rotor,
        airframe,
        performance
    } = config;

    const W   = lbsToNewtons(weightLbs);
    const IAS = knotsToMs(airspeedKnots);
    const wind = windComponents(windSpeedKnots, windDirectionDeg);

    // Entry azimuth: if the aircraft enters heading H (nav degrees),
    // and ground track tangent heading at psi is (psi + 90°),
    // then entry psi = H - 90° (in radians).
    const entryPsi = degreesToRadians(headingDeg) - Math.PI / 2;

    const numSteps = 360;
    const dpsi = (2 * Math.PI) / numSteps;

    const results = {
        numSteps,
        dt: [],               // per-step time delta (seconds)
        totalTime: 0,
        turnRadiusM,
        wind,
        airspeedKnots,
        x: [],
        y: [],
        psi: [],
        groundSpeed: [],      // m/s at each step
        groundTrackHeading: [],
        bankAngle: [],        // radians
        loadFactors: [],
        power: [],
        torque: []
    };

    let totalTime = 0;

    for (let i = 0; i < numSteps; i++) {
        const psi = entryPsi + i * dpsi;

        // Position on ground circle
        results.x.push(turnRadiusM * Math.sin(psi));
        results.y.push(turnRadiusM * Math.cos(psi));
        results.psi.push(psi);

        // Ground track heading (nav convention: CW from north)
        // Tangent direction (cos(psi), -sin(psi)) → heading = atan2(cos(psi), -sin(psi))
        results.groundTrackHeading.push(Math.atan2(Math.cos(psi), -Math.sin(psi)));

        // Ground speed from constant IAS + wind
        const Vg = groundSpeedFromIAS(IAS, wind, psi);
        results.groundSpeed.push(Vg);

        // Bank angle varies with ground speed
        const bank = bankAngleForTurn(Vg, turnRadiusM);
        results.bankAngle.push(bank);
        results.loadFactors.push(loadFactor(bank));

        // Power computed using the constant IAS (aero forces depend on airspeed)
        const pwr = powerComponents({
            weightN: W,
            airspeedMs: IAS,
            bankAngleRad: bank,
            rotor,
            airframe
        });
        results.power.push(pwr);

        // Torque %
        results.torque.push(
            torquePercent(pwr.total, performance.maxContinuousPower, rotor.omega)
        );

        // Time for this angular step: dt = dpsi * R / Vg
        const dt = Vg > 0.1 ? (dpsi * turnRadiusM / Vg) : 1;
        results.dt.push(dt);
        totalTime += dt;
    }

    results.totalTime = totalTime;

    // Summary stats
    const banksDeg = results.bankAngle.map(b => b * 180 / Math.PI);
    results.minBankDeg = Math.min(...banksDeg);
    results.maxBankDeg = Math.max(...banksDeg);
    results.minGroundSpeedKnots = msToKnots(Math.min(...results.groundSpeed));
    results.maxGroundSpeedKnots = msToKnots(Math.max(...results.groundSpeed));
    results.minTorque = Math.min(...results.torque);
    results.maxTorque = Math.max(...results.torque);

    return results;
}
