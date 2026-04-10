import { lbsToNewtons, knotsToMs, msToKnots, degreesToRadians } from './utils.js';

const RHO = 1.225;   // air density kg/m^3  (sea level ISA)
const G   = 9.81;    // gravitational acceleration m/s^2

// Overhead factors applied to all helicopter types
const TAIL_ROTOR_POWER_FRACTION = 0.10;  // tail rotor ≈ 10% of main rotor power
const ACCESSORY_POWER_FRACTION  = 0.03;  // hydraulics, generators, etc. ≈ 3% of max continuous
const TRANSMISSION_EFFICIENCY   = 0.975; // 2.5% gearbox losses

// Forward-flight empirical correction factor.
// Accounts for compressibility on advancing blade tip, non-uniform inflow,
// blade-fuselage interference, reverse flow, and hub drag — effects that
// simplified momentum/blade-element theory consistently underestimates.
// Calibrated against UH-60 PPC data and pilot-reported torque values.
const FWD_FLIGHT_K = 4.0;

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
// The aircraft flies a circular ground track; direction determines tangent.
// dir = 1 for left (CCW), -1 for right (CW).
// Returns ground speed in m/s (positive), or 0 if IAS < crosswind component.
export function groundSpeedFromIAS(iasMps, wind, psi, dir) {
    // Ground track tangent direction
    // CCW (left): (-cos(psi), sin(psi)).  CW (right): (cos(psi), -sin(psi)).
    const tx = -dir * Math.cos(psi);
    const ty = dir * Math.sin(psi);

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
// Returns main rotor components AND total engine power required
// (including tail rotor, accessories, transmission losses, and
// forward-flight corrections for compressibility/interference).
export function powerComponents(params) {
    const { weightN, airspeedMs, bankAngleRad, rotor, airframe, maxContinuousPowerKW, vneKnots } = params;
    const { radius, omega, solidity } = rotor;
    const { bladeProfileDragCoeff: Cd0, inducedPowerFactor: kFactor, flatPlateArea: Seq } = airframe;

    const n = loadFactor(bankAngleRad);
    const thrust = weightN * n;

    // Induced velocity accounts for increased thrust in the turn
    const vi0 = hoverInducedVelocity(thrust, radius);
    const vi = forwardInducedVelocity(vi0, airspeedMs);

    // Induced power: k * T * vi
    const induced = kFactor * thrust * vi;

    // Profile power with advance ratio correction:
    // Pp = Pp0 * (1 + 4.65 * mu²) where mu = V / (omega * R)
    const A = Math.PI * radius * radius;
    const vtip = omega * radius;
    const mu = airspeedMs / vtip;   // advance ratio
    const profileHover = (solidity * Cd0 / 8) * RHO * A * Math.pow(vtip, 3);
    const profile = profileHover * (1 + 4.65 * mu * mu);

    // Parasite power
    const parasite = 0.5 * RHO * Math.pow(airspeedMs, 3) * Seq;

    // Main rotor total (first-principles)
    const mainRotorBase = induced + profile + parasite;

    // Forward-flight correction for effects not captured by simplified model:
    // compressibility on advancing blade tip, non-uniform inflow,
    // blade-fuselage interference, reverse flow region, hub drag.
    // Correction peaks in the mid-speed range and tapers near Vne
    // where parasite power (modeled correctly) dominates.
    const vne = knotsToMs(vneKnots || 200);
    const muMax = vne / vtip;
    const fwdCorrection = FWD_FLIGHT_K * mu * Math.max(0, 1 - mu / muMax);
    const mainRotor = mainRotorBase * (1 + fwdCorrection);

    // Tail rotor power: ~10% of main rotor power
    const tailRotor = TAIL_ROTOR_POWER_FRACTION * mainRotor;

    // Accessories: ~3% of max continuous power (hydraulics, generators, etc.)
    const accessories = ACCESSORY_POWER_FRACTION * (maxContinuousPowerKW || 0) * 1000;

    // Total power before transmission losses
    const subtotal = mainRotor + tailRotor + accessories;

    // Engine output must overcome transmission losses
    const total = subtotal / TRANSMISSION_EFFICIENCY;

    return { induced, profile, parasite, mainRotor, tailRotor, accessories, total };
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
        turnDirection,   // 'left' or 'right'
        rotor,
        airframe,
        performance
    } = config;

    const W   = lbsToNewtons(weightLbs);
    const IAS = knotsToMs(airspeedKnots);
    const wind = windComponents(windSpeedKnots, windDirectionDeg);

    // dir = 1 for left (CCW on canvas), -1 for right (CW on canvas)
    const dir = turnDirection === 'right' ? -1 : 1;

    // Entry azimuth: solve for psi such that tangent heading matches entry heading.
    // Tangent heading = atan2(-dir*cos(psi), dir*sin(psi))
    // For left (dir=1):  psi = atan2(cos(H), -sin(H))
    // For right (dir=-1): psi = atan2(-cos(H), sin(H)) = H - PI/2 in radians
    const headingRad = degreesToRadians(headingDeg);
    const entryPsi = Math.atan2(dir * Math.cos(headingRad), -dir * Math.sin(headingRad));

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
        // Step psi: negative for left (CCW), positive for right (CW)
        const psi = entryPsi - dir * i * dpsi;

        // Position on ground circle
        results.x.push(turnRadiusM * Math.sin(psi));
        results.y.push(turnRadiusM * Math.cos(psi));
        results.psi.push(psi);

        // Ground track heading (nav convention: CW from north)
        results.groundTrackHeading.push(Math.atan2(-dir * Math.cos(psi), dir * Math.sin(psi)));

        // Ground speed from constant IAS + wind
        const Vg = groundSpeedFromIAS(IAS, wind, psi, dir);
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
            airframe,
            maxContinuousPowerKW: performance.maxContinuousPower,
            vneKnots: performance.vne
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

// Run an orbit followed by a straight-in approach to the center point.
// The helicopter orbits until reaching a break point near the approach
// heading, then turns to the inbound heading and flies straight to the
// point while decelerating to near-hover.
export function runApproachToPoint(config) {
    const {
        weightLbs,
        headingDeg,
        windDirectionDeg,
        windSpeedKnots,
        turnRadiusM,
        airspeedKnots,
        turnDirection,
        approachHeadingDeg,
        rotor,
        airframe,
        performance
    } = config;

    const W    = lbsToNewtons(weightLbs);
    const IAS  = knotsToMs(airspeedKnots);
    const wind = windComponents(windSpeedKnots, windDirectionDeg);
    const dir  = turnDirection === 'right' ? -1 : 1;

    const headingRad  = degreesToRadians(headingDeg);
    const approachRad = degreesToRadians(approachHeadingDeg);
    const entryPsi    = Math.atan2(dir * Math.cos(headingRad), -dir * Math.sin(headingRad));

    // Approach start: the point on the orbit where the inbound line to
    // center begins. At this psi, heading toward center = approachHeadingDeg.
    // Vector from position to center has heading psi + PI, so psi = approachRad - PI.
    const approachPsi = approachRad - Math.PI;

    // Orbit angular distance from entry to break point
    let orbitSweep = (entryPsi - approachPsi) * dir;
    while (orbitSweep <= 0) orbitSweep += 2 * Math.PI;
    if (orbitSweep < degreesToRadians(10)) orbitSweep += 2 * Math.PI;

    const dpsi = degreesToRadians(1);
    const orbitSteps = Math.round(orbitSweep / dpsi);

    // Straight-in: from break point on circle to center, distance = turnRadiusM
    const straightSteps = 180;
    const stepDist = turnRadiusM / straightSteps;
    const terminalSpeed = knotsToMs(10);

    // Break point position
    const breakX = turnRadiusM * Math.sin(approachPsi);
    const breakY = turnRadiusM * Math.cos(approachPsi);

    // Inbound direction unit vector (toward center from break point)
    const inboundX = -Math.sin(approachPsi);  // = sin(approachRad)... normalized
    const inboundY = -Math.cos(approachPsi);
    // Normalize (already unit length since break is on unit circle * R)

    const numSteps = orbitSteps + straightSteps;

    const results = {
        numSteps,
        dt: [],
        totalTime: 0,
        turnRadiusM,
        wind,
        airspeedKnots,
        isApproach: true,
        approachStartStep: orbitSteps,
        x: [], y: [], psi: [],
        groundSpeed: [],
        groundTrackHeading: [],
        bankAngle: [],
        loadFactors: [],
        power: [],
        torque: [],
        phase: [],
        radius: [],
        ias: []
    };

    let totalTime = 0;

    // --- Orbit phase ---
    for (let i = 0; i < orbitSteps; i++) {
        const psi = entryPsi - dir * i * dpsi;

        results.x.push(turnRadiusM * Math.sin(psi));
        results.y.push(turnRadiusM * Math.cos(psi));
        results.psi.push(psi);
        results.phase.push('orbit');
        results.radius.push(turnRadiusM);
        results.ias.push(msToKnots(IAS));
        results.groundTrackHeading.push(Math.atan2(-dir * Math.cos(psi), dir * Math.sin(psi)));

        const Vg = groundSpeedFromIAS(IAS, wind, psi, dir);
        results.groundSpeed.push(Vg);

        const bank = bankAngleForTurn(Vg, turnRadiusM);
        results.bankAngle.push(bank);
        results.loadFactors.push(loadFactor(bank));

        const pwr = powerComponents({
            weightN: W, airspeedMs: IAS, bankAngleRad: bank,
            rotor, airframe,
            maxContinuousPowerKW: performance.maxContinuousPower,
            vneKnots: performance.vne
        });
        results.power.push(pwr);
        results.torque.push(torquePercent(pwr.total, performance.maxContinuousPower, rotor.omega));

        const dt = Vg > 0.1 ? (dpsi * turnRadiusM / Vg) : 1;
        results.dt.push(dt);
        totalTime += dt;
    }

    // --- Straight-in approach phase ---
    // Tailwind component along the inbound heading
    const tailwindComponent = wind.x * Math.sin(approachRad) + wind.y * Math.cos(approachRad);

    for (let i = 0; i < straightSteps; i++) {
        const t = i / straightSteps; // 0 at break, ~1 at center
        const dist = turnRadiusM * (1 - t); // distance remaining to center

        // Position: linear interpolation from break point to center
        const px = breakX + inboundX * turnRadiusM * t;
        const py = breakY + inboundY * turnRadiusM * t;

        results.x.push(px);
        results.y.push(py);
        results.psi.push(approachPsi);
        results.phase.push('approach');
        results.radius.push(dist);
        results.groundTrackHeading.push(approachRad);

        // IAS decreases linearly from cruise to terminal
        const iasMs = IAS * (1 - t) + terminalSpeed * t;
        results.ias.push(msToKnots(iasMs));

        // Ground speed = IAS + tailwind (straight flight)
        const Vg = Math.max(0.5, iasMs + tailwindComponent);
        results.groundSpeed.push(Vg);

        // Straight flight: no bank
        results.bankAngle.push(0);
        results.loadFactors.push(1);

        const pwr = powerComponents({
            weightN: W, airspeedMs: iasMs, bankAngleRad: 0,
            rotor, airframe,
            maxContinuousPowerKW: performance.maxContinuousPower,
            vneKnots: performance.vne
        });
        results.power.push(pwr);
        results.torque.push(torquePercent(pwr.total, performance.maxContinuousPower, rotor.omega));

        const dt = Vg > 0.1 ? (stepDist / Vg) : 1;
        results.dt.push(dt);
        totalTime += dt;
    }

    results.totalTime = totalTime;

    const banksDeg = results.bankAngle.map(b => b * 180 / Math.PI);
    results.minBankDeg = Math.min(...banksDeg);
    results.maxBankDeg = Math.max(...banksDeg);
    results.minGroundSpeedKnots = msToKnots(Math.min(...results.groundSpeed));
    results.maxGroundSpeedKnots = msToKnots(Math.max(...results.groundSpeed));
    results.minTorque = Math.min(...results.torque);
    results.maxTorque = Math.max(...results.torque);

    return results;
}
