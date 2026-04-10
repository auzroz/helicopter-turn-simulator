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

// Run an orbit followed by a lead-in turn and straight-in approach.
// Phase 1: Orbit from entry heading to the break point
// Phase 2: 90° lead-in turn at ~25° bank, curving from orbit tangent to inbound heading
// Phase 3: Straight-in to center, decelerating to near-hover
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

    // Break point: where the inbound heading line meets the orbit circle.
    // At this azimuth, the vector to center has heading = approachRad.
    const approachPsi = approachRad - Math.PI;

    // Orbit angular distance from entry to break point
    let orbitSweep = (entryPsi - approachPsi) * dir;
    while (orbitSweep <= 0) orbitSweep += 2 * Math.PI;
    if (orbitSweep < degreesToRadians(10)) orbitSweep += 2 * Math.PI;

    const dpsi = degreesToRadians(1);
    const orbitSteps = Math.round(orbitSweep / dpsi);

    // Lead-in turn: 90° heading change from orbit tangent to inbound heading.
    // At the break point, orbit tangent is always perpendicular to the inbound heading.
    // The turn continues in the same direction (left for left orbit, right for right).
    const LEADIN_BANK_DEG = 25;
    const leadinBankRad = degreesToRadians(LEADIN_BANK_DEG);
    const leadinTurnSteps = 90; // 1° heading change per step
    const headingStepRad = degreesToRadians(1);

    // Orbit tangent heading at break point
    const orbitHeadingAtBreak = Math.atan2(-dir * Math.cos(approachPsi), dir * Math.sin(approachPsi));

    // Straight-in: distance depends on where the lead-in turn ends
    // We'll compute straight-in after generating the lead-in turn path
    const terminalSpeed = knotsToMs(10);

    const results = {
        numSteps: 0,  // filled after all phases
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

    function pushStep(px, py, heading, iasMs, bank, phase) {
        results.x.push(px);
        results.y.push(py);
        results.psi.push(heading);
        results.groundTrackHeading.push(heading);
        results.phase.push(phase);
        results.ias.push(msToKnots(iasMs));
        results.bankAngle.push(bank);
        results.loadFactors.push(loadFactor(bank));

        // Wind-adjusted ground speed: tailwind component along heading
        const tailwind = wind.x * Math.sin(heading) + wind.y * Math.cos(heading);
        const Vg = Math.max(0.5, iasMs + tailwind);
        results.groundSpeed.push(Vg);

        // Radius estimate for display (orbit R during orbit, turn radius during lead-in)
        const r = phase === 'orbit' ? turnRadiusM :
                  phase === 'turn'  ? (iasMs * iasMs / (G * Math.tan(Math.max(bank, 0.01)))) :
                  Math.sqrt(px * px + py * py);
        results.radius.push(r);

        const pwr = powerComponents({
            weightN: W, airspeedMs: iasMs, bankAngleRad: bank,
            rotor, airframe,
            maxContinuousPowerKW: performance.maxContinuousPower,
            vneKnots: performance.vne
        });
        results.power.push(pwr);
        results.torque.push(torquePercent(pwr.total, performance.maxContinuousPower, rotor.omega));

        return Vg;
    }

    // --- Phase 1: Orbit ---
    for (let i = 0; i < orbitSteps; i++) {
        const psi = entryPsi - dir * i * dpsi;
        const px = turnRadiusM * Math.sin(psi);
        const py = turnRadiusM * Math.cos(psi);
        const heading = Math.atan2(-dir * Math.cos(psi), dir * Math.sin(psi));

        const Vg = groundSpeedFromIAS(IAS, wind, psi, dir);
        const bank = bankAngleForTurn(Vg, turnRadiusM);

        // Push orbit step directly (use orbit-specific ground speed calc)
        results.x.push(px);
        results.y.push(py);
        results.psi.push(psi);
        results.groundTrackHeading.push(heading);
        results.phase.push('orbit');
        results.ias.push(msToKnots(IAS));
        results.bankAngle.push(bank);
        results.loadFactors.push(loadFactor(bank));
        results.groundSpeed.push(Vg);
        results.radius.push(turnRadiusM);

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

    // --- Phase 2: Lead-in turn (90° heading change) ---
    // Position is integrated by advancing along the current heading each step
    let curX = turnRadiusM * Math.sin(approachPsi);
    let curY = turnRadiusM * Math.cos(approachPsi);
    let curHeading = orbitHeadingAtBreak;

    for (let i = 0; i < leadinTurnSteps; i++) {
        const t = i / leadinTurnSteps;
        // IAS begins decelerating gently during the turn (cruise to ~80% cruise)
        const iasMs = IAS * (1 - 0.2 * t);

        const Vg = pushStep(curX, curY, curHeading, iasMs, leadinBankRad, 'turn');

        // Advance position along current heading
        const turnRadius = iasMs * iasMs / (G * Math.tan(leadinBankRad));
        const dt = turnRadius > 0.1 ? (headingStepRad * turnRadius / Vg) : 0.5;
        const dist = Vg * dt;
        curX += dist * Math.sin(curHeading);
        curY += dist * Math.cos(curHeading);

        results.dt.push(dt);
        totalTime += dt;

        // Advance heading (left turn = -dir because dir=1 means heading decreases)
        curHeading -= dir * headingStepRad;
    }

    // --- Phase 3: Straight-in to center ---
    // Distance remaining from current position to center
    const distToCenter = Math.sqrt(curX * curX + curY * curY);
    const straightSteps = Math.max(30, Math.round(distToCenter / 3)); // ~3m per step
    const iasAtStraightStart = IAS * 0.8; // speed after lead-in deceleration

    for (let i = 0; i < straightSteps; i++) {
        const t = i / straightSteps;
        const px = curX * (1 - t);
        const py = curY * (1 - t);

        const iasMs = iasAtStraightStart * (1 - t) + terminalSpeed * t;
        const Vg = pushStep(px, py, approachRad, iasMs, 0, 'approach');

        const stepLen = distToCenter / straightSteps;
        const dt = Vg > 0.1 ? (stepLen / Vg) : 1;
        results.dt.push(dt);
        totalTime += dt;
    }

    results.numSteps = results.x.length;
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
