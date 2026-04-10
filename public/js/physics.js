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
    const terminalSpeed = knotsToMs(20);            // 20kt at the point
    const DECEL_DIST = 800;                         // decelerate over last 800m max

    // Coordinate frame aligned with approach
    const ax = Math.sin(H), ay = Math.cos(H);     // along-axis (toward target)
    const cx = -dir * Math.cos(H), cy = dir * Math.sin(H); // cross-axis (pattern side)

    function toWorld(along, cross) {
        return [along * ax + cross * cx, along * ay + cross * cy];
    }

    // Pattern turn radius from IAS at 25° bank
    let r = IAS * IAS / (G * Math.tan(degreesToRadians(25)));
    if (r * 2 > patternWidth) r = patternWidth / 2; // ensure turns fit
    const turnBankRad = Math.atan(IAS * IAS / (G * r));

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

    // Helper: push one step of straight flight
    function pushStraight(px, py, heading, iasMs, phase) {
        results.x.push(px);
        results.y.push(py);
        results.psi.push(heading);
        results.groundTrackHeading.push(heading);
        results.phase.push(phase);
        results.ias.push(msToKnots(iasMs));
        results.bankAngle.push(0);
        results.loadFactors.push(1);
        const tailwind = wind.x * Math.sin(heading) + wind.y * Math.cos(heading);
        const Vg = Math.max(0.5, iasMs + tailwind);
        results.groundSpeed.push(Vg);
        results.radius.push(0);
        const pwr = powerComponents({
            weightN: W, airspeedMs: iasMs, bankAngleRad: 0,
            rotor, airframe,
            maxContinuousPowerKW: performance.maxContinuousPower,
            vneKnots: performance.vne
        });
        results.power.push(pwr);
        results.torque.push(torquePercent(pwr.total, performance.maxContinuousPower, rotor.omega));
        return Vg;
    }

    // Helper: push one step of turning flight
    function pushTurn(px, py, heading, iasMs, bankRad, phase) {
        results.x.push(px);
        results.y.push(py);
        results.psi.push(heading);
        results.groundTrackHeading.push(heading);
        results.phase.push(phase);
        results.ias.push(msToKnots(iasMs));
        results.bankAngle.push(bankRad);
        results.loadFactors.push(loadFactor(bankRad));
        const tailwind = wind.x * Math.sin(heading) + wind.y * Math.cos(heading);
        const Vg = Math.max(0.5, iasMs + tailwind);
        results.groundSpeed.push(Vg);
        results.radius.push(r);
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

    // --- Phase 2: Base turn (90°) ---
    let curX, curY;
    [curX, curY] = toWorld(-approachDist, patternWidth);
    let curHeading = downwindHdg;

    for (let i = 0; i < 90; i++) {
        const Vg = pushTurn(curX, curY, curHeading, IAS, turnBankRad, 'turn');
        const tr = IAS * IAS / (G * Math.tan(turnBankRad));
        const dt = tr > 0.1 ? (dhdg * tr / Vg) : 0.5;
        const dist = Vg * dt;
        curX += dist * Math.sin(curHeading);
        curY += dist * Math.cos(curHeading);
        curHeading -= dir * dhdg;
        results.dt.push(dt);
        totalTime += dt;
    }

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

    // --- Phase 4: Final turn (90°) ---
    // Slight deceleration during turn (cruise to ~90%)
    curHeading = baseHdg;

    for (let i = 0; i < 90; i++) {
        const t = i / 90;
        const iasMs = IAS * (1 - 0.1 * t);
        const Vg = pushTurn(curX, curY, curHeading, iasMs, turnBankRad, 'turn');
        const tr = iasMs * iasMs / (G * Math.tan(turnBankRad));
        const dt = tr > 0.1 ? (dhdg * tr / Vg) : 0.5;
        const dist = Vg * dt;
        curX += dist * Math.sin(curHeading);
        curY += dist * Math.cos(curHeading);
        curHeading -= dir * dhdg;
        results.dt.push(dt);
        totalTime += dt;
    }

    // --- Phase 5: Final approach (straight to center, decelerating) ---
    // Deceleration from ~90% cruise to terminal over the last DECEL_DIST meters.
    // If final is longer than DECEL_DIST, cruise first then decelerate.
    // Target: 800m in ~30-45 seconds.
    const distToCenter = Math.sqrt(curX * curX + curY * curY);
    const finalSteps = Math.max(30, Math.round(distToCenter / 5));
    const finalStepLen = distToCenter / finalSteps;
    const iasAtFinal = IAS * 0.9; // speed after final turn
    const decelStart = Math.max(0, distToCenter - DECEL_DIST); // distance from curPos where decel begins

    for (let i = 0; i < finalSteps; i++) {
        const t = i / finalSteps;
        const distFromStart = distToCenter * t;
        const px = curX * (1 - t);
        const py = curY * (1 - t);

        let iasMs;
        if (distFromStart < decelStart) {
            // Cruise portion before deceleration zone
            iasMs = iasAtFinal;
        } else {
            // Deceleration zone: smooth decel over last DECEL_DIST meters
            const dt2 = (distFromStart - decelStart) / (distToCenter - decelStart);
            // Use sqrt profile: decelerates briskly at first, eases near the end
            const blend = Math.sqrt(dt2);
            iasMs = iasAtFinal * (1 - blend) + terminalSpeed * blend;
        }

        const Vg = pushStraight(px, py, finalHdg, iasMs, 'final');
        const dt = Vg > 0.1 ? (finalStepLen / Vg) : 1;
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
