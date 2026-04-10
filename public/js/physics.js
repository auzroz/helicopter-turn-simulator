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

// Solve for ground speed given IAS, wind, and flight heading.
// General form: works for straight flight and turns.
export function groundSpeedForHeading(iasMps, wind, heading) {
    const tx = Math.sin(heading);
    const ty = Math.cos(heading);
    const h = tx * wind.x + ty * wind.y;
    const discriminant = iasMps * iasMps - wind.magnitude * wind.magnitude + h * h;
    if (discriminant < 0) return Math.max(0.5, h);
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

// Run a traffic-pattern approach to the center point.
// Downwind → base turn → base leg (if needed) → final turn → final approach.
//
// Geometry (in along/cross frame where along points toward the target,
// cross points toward the pattern side):
//   Downwind:   (0, W) → (-L, W)               heading H+180°
//   Base turn:  90° arc, radius r               center (-L, W-r)
//   Base leg:   (-L-r, W-r) → (-L-r, r)         heading H±90°
//   Final turn: 90° arc, radius r               center (-L, r)
//   Final:      (-L, 0) → (0, 0)                heading H, decelerating
export function runApproachPattern(config) {
    const {
        weightLbs,
        windDirectionDeg,
        windSpeedKnots,
        turnRadiusM,       // reused as pattern width W
        airspeedKnots,
        turnDirection,
        approachHeadingDeg,
        approachDistanceM,
        rotor,
        airframe,
        performance
    } = config;

    const W    = lbsToNewtons(weightLbs);
    const IAS  = knotsToMs(airspeedKnots);
    const wind = windComponents(windSpeedKnots, windDirectionDeg);
    const dir  = turnDirection === 'right' ? -1 : 1;

    const H = degreesToRadians(approachHeadingDeg);
    const patternWidth = turnRadiusM;              // offset from final course to downwind
    const approachDist = approachDistanceM || 1000; // length of final approach
    // 3:1 approach profile (altitude/airspeed = 3) at ~8° approach angle:
    // altitude(ft) = 3 × airspeed(kt)
    // distance(m) = altitude(ft) × 0.3048 / tan(8°) = airspeed × 0.9144 / tan(8°)
    // airspeed(kt) = distance(m) × tan(8°) / 0.9144 = distance / 6.5
    const PROFILE_FACTOR = 6.5;  // meters per knot at 8° approach angle

    // Coordinate frame aligned with approach
    const ax = Math.sin(H), ay = Math.cos(H);     // along-axis (toward target)
    const cx = -dir * Math.cos(H), cy = dir * Math.sin(H); // cross-axis (pattern side)

    function toWorld(along, cross) {
        return [along * ax + cross * cx, along * ay + cross * cy];
    }

    // Pattern turn radius from IAS at 25° bank
    let r = IAS * IAS / (G * Math.tan(degreesToRadians(25)));
    if (r * 2 > patternWidth) r = patternWidth / 2; // ensure turns fit
    // Headings
    const downwindHdg = H + Math.PI;
    const baseHdg = H + Math.PI - dir * Math.PI / 2; // perpendicular toward final course
    const finalHdg = H;

    // Base leg length (0 if turns connect directly)
    const baseLegLen = Math.max(0, patternWidth - 2 * r);

    const results = {
        numSteps: 0,
        dt: [],
        totalTime: 0,
        turnRadiusM: patternWidth,
        wind,
        airspeedKnots,
        isApproach: true,
        approachStartStep: 0,
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
    const dhdg = degreesToRadians(1);

    // Shared power computation — same logic as orbit mode
    function computeAndPush(px, py, heading, iasMs, bankRad, turnR, phase) {
        const Vg = groundSpeedForHeading(iasMs, wind, heading);

        results.x.push(px);
        results.y.push(py);
        results.psi.push(heading);
        results.groundTrackHeading.push(heading);
        results.phase.push(phase);
        results.ias.push(msToKnots(iasMs));
        results.bankAngle.push(bankRad);
        results.loadFactors.push(loadFactor(bankRad));
        results.groundSpeed.push(Vg);
        results.radius.push(turnR);

        const pwr = powerComponents({
            weightN: W, airspeedMs: iasMs, bankAngleRad: bankRad,
            rotor, airframe,
            maxContinuousPowerKW: performance.maxContinuousPower,
            vneKnots: performance.vne
        });
        results.power.push(pwr);
        results.torque.push(torquePercent(pwr.total, performance.maxContinuousPower, rotor.omega));
        return Vg;
    }

    // Helper: push one step of straight flight
    function pushStraight(px, py, heading, iasMs, phase) {
        return computeAndPush(px, py, heading, iasMs, 0, 0, phase);
    }

    // Generate a 90° arc with geometric positions on the circle.
    // Same approach as the orbit: positions are constrained to the arc,
    // bank angle is derived from what's needed to hold the ground track.
    function generateArc(startX, startY, startHdg, iasStart, iasEnd, phase) {
        // Turn center: perpendicular to heading, on the inside of the turn
        const centerX = startX + r * (-dir * Math.cos(startHdg));
        const centerY = startY + r * (dir * Math.sin(startHdg));

        for (let i = 0; i < 90; i++) {
            const hdg = startHdg - dir * i * dhdg;
            const t = i / 90;
            const iasMs = iasStart * (1 - t) + iasEnd * t;

            // Geometric position on the arc (just like orbit positions on a circle)
            const px = centerX + r * (dir * Math.cos(hdg));
            const py = centerY + r * (-dir * Math.sin(hdg));

            // Bank derived from ground speed and radius (same as orbit)
            const Vg = groundSpeedForHeading(iasMs, wind, hdg);
            const fullBank = bankAngleForTurn(Vg, r);

            // Apply roll-in/roll-out ramp
            const ramp = rampedBank(i, 90, 1.0);
            const bankRad = fullBank * ramp;

            computeAndPush(px, py, hdg, iasMs, bankRad, r, phase);

            // dt from arc segment length and ground speed
            const dt = Vg > 0.1 ? (dhdg * r / Vg) : 0.5;
            results.dt.push(dt);
            totalTime += dt;
        }

        // Return end position and heading
        const endHdg = startHdg - dir * 90 * dhdg;
        const endX = centerX + r * (dir * Math.cos(endHdg));
        const endY = centerY + r * (-dir * Math.sin(endHdg));
        return [endX, endY, endHdg];
    }

    // --- Phase 1: Downwind leg ---
    // From abeam the point (along=0) to base turn initiation (along=-approachDist)
    const downwindSteps = Math.max(30, Math.round(approachDist / 5));
    const downwindStepLen = approachDist / downwindSteps;

    for (let i = 0; i <= downwindSteps; i++) {
        const along = -(approachDist * i / downwindSteps);
        const [px, py] = toWorld(along, patternWidth);
        const Vg = pushStraight(px, py, downwindHdg, IAS, 'downwind');
        const dt = Vg > 0.1 ? (downwindStepLen / Vg) : 1;
        results.dt.push(dt);
        totalTime += dt;
    }

    // Bank angle ramp: gradual roll-in over first RAMP degrees,
    // full bank in the middle, gradual roll-out over last RAMP degrees.
    const RAMP_DEG = 15;
    function rampedBank(step, totalSteps, maxBank) {
        if (step < RAMP_DEG) return maxBank * (step / RAMP_DEG);
        if (step > totalSteps - RAMP_DEG) return maxBank * ((totalSteps - step) / RAMP_DEG);
        return maxBank;
    }

    // --- Phase 2: Base turn (90° arc, geometric positions) ---
    let curX, curY, curHeading;
    const baseStart = toWorld(-approachDist, patternWidth);
    [curX, curY, curHeading] = generateArc(baseStart[0], baseStart[1], downwindHdg, IAS, IAS, 'turn');

    // --- Phase 3: Base leg (if needed) ---
    // Maintain cruise speed through base — no deceleration yet
    if (baseLegLen > 0) {
        const baseSteps = Math.max(10, Math.round(baseLegLen / 5));
        const baseStepLen = baseLegLen / baseSteps;
        const [baseStartX, baseStartY] = [curX, curY];
        const bx = Math.sin(baseHdg), by = Math.cos(baseHdg);

        for (let i = 0; i < baseSteps; i++) {
            const t = i / baseSteps;
            const px = baseStartX + bx * baseLegLen * t;
            const py = baseStartY + by * baseLegLen * t;
            const Vg = pushStraight(px, py, baseHdg, IAS, 'base');
            const dt = Vg > 0.1 ? (baseStepLen / Vg) : 1;
            results.dt.push(dt);
            totalTime += dt;
        }
        curX = baseStartX + bx * baseLegLen;
        curY = baseStartY + by * baseLegLen;
    }

    // --- Phase 4: Final turn (90° arc, geometric positions) ---
    // Slight deceleration during turn (cruise to ~90%)
    [curX, curY, curHeading] = generateArc(curX, curY, baseHdg, IAS, IAS * 0.9, 'turn');

    // --- Phase 5: Final approach (straight to center, 3:1 profile) ---
    // Standard helicopter approach: GS(kt) = distance(m) / 9.14
    // Speed is proportional to distance from the point, reaching 0 at the point.
    // If the aircraft is faster than the profile at rollout, cruise until the
    // profile catches up, then follow the profile to the point.
    const distToCenter = Math.sqrt(curX * curX + curY * curY);
    const finalSteps = Math.max(30, Math.round(distToCenter / 5));
    const finalStepLen = distToCenter / finalSteps;
    const iasAfterTurn = IAS * 0.9; // speed after final turn

    for (let i = 0; i < finalSteps; i++) {
        const t = i / finalSteps;
        const distRemaining = distToCenter * (1 - t);
        const px = curX * (1 - t);
        const py = curY * (1 - t);

        // 3:1 profile speed at this distance
        const profileSpeedKt = distRemaining / PROFILE_FACTOR;
        const profileSpeedMs = knotsToMs(profileSpeedKt);

        // Use the lower of current approach speed and profile speed.
        // Cruise until profile catches up, then follow profile to the point.
        // Minimum 5kt (~2.6 m/s) to represent the final hover transition.
        const minSpeed = knotsToMs(5);
        const iasMs = Math.min(iasAfterTurn, Math.max(profileSpeedMs, minSpeed));

        const Vg = pushStraight(px, py, finalHdg, iasMs, 'final');
        const dt = Vg > 0.5 ? (finalStepLen / Vg) : 0.5;
        results.dt.push(dt);
        totalTime += dt;
    }

    results.numSteps = results.x.length;
    results.totalTime = totalTime;
    results.approachStartStep = downwindSteps;

    const banksDeg = results.bankAngle.map(b => b * 180 / Math.PI);
    results.minBankDeg = Math.min(...banksDeg);
    results.maxBankDeg = Math.max(...banksDeg);
    results.minGroundSpeedKnots = msToKnots(Math.min(...results.groundSpeed));
    results.maxGroundSpeedKnots = msToKnots(Math.max(...results.groundSpeed));
    results.minTorque = Math.min(...results.torque);
    results.maxTorque = Math.max(...results.torque);

    return results;
}
