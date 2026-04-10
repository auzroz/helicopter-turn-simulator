// Unit conversions
export function lbsToNewtons(lbs) {
    return lbs * 4.44822;
}

export function newtonsToLbs(n) {
    return n / 4.44822;
}

export function knotsToMs(knots) {
    return knots * 0.514444;
}

export function msToKnots(ms) {
    return ms / 0.514444;
}

export function degreesToRadians(deg) {
    return deg * Math.PI / 180;
}

export function radiansToDegrees(rad) {
    return rad * 180 / Math.PI;
}

export function kWToWatts(kw) {
    return kw * 1000;
}

// Math helpers
export function clamp(value, min, max) {
    return Math.max(min, Math.min(max, value));
}

export function lerp(a, b, t) {
    return a + (b - a) * t;
}

// Color palette for multiple simulations
const SIM_COLORS = ['#e74c3c', '#3498db', '#2ecc71', '#f39c12'];

export function getSimColor(index) {
    return SIM_COLORS[index % SIM_COLORS.length];
}

export const MAX_SIMULATIONS = 4;
