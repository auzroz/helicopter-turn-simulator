import { degreesToRadians } from './utils.js';

// Setup canvas for high-DPI
export function setupCanvas(canvas) {
    const dpr = window.devicePixelRatio || 1;
    const rect = canvas.getBoundingClientRect();
    canvas.width = rect.width * dpr;
    canvas.height = rect.height * dpr;
    const ctx = canvas.getContext('2d');
    ctx.scale(dpr, dpr);
    return { ctx, width: rect.width, height: rect.height };
}

// Compute a uniform scale that fits all simulations on the canvas.
export function computeScale(simulations, canvasWidth, canvasHeight) {
    let maxRadius = 100;
    for (const sim of simulations) {
        if (sim.results) {
            maxRadius = Math.max(maxRadius, sim.results.turnRadiusM);
        }
    }
    const padding = 60;
    const size = Math.min(canvasWidth, canvasHeight);
    return (size / 2 - padding) / maxRadius;
}

// Draw the full scene: compass, wind, flight paths, helicopters.
export function drawScene(canvas, simulations) {
    const { ctx, width, height } = setupCanvas(canvas);
    const cx = width / 2;
    const cy = height / 2;
    const scale = computeScale(simulations, width, height);

    ctx.clearRect(0, 0, width, height);

    drawCompassRose(ctx, cx, cy, Math.min(width, height) / 2 - 15);

    // Ground target (center point)
    ctx.fillStyle = '#ffffff';
    ctx.beginPath();
    ctx.arc(cx, cy, 4, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = 'rgba(255,255,255,0.3)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(cx - 10, cy);
    ctx.lineTo(cx + 10, cy);
    ctx.moveTo(cx, cy - 10);
    ctx.lineTo(cx, cy + 10);
    ctx.stroke();

    // Draw each simulation
    for (const sim of simulations) {
        if (!sim.results) continue;
        drawFlightPath(ctx, sim, cx, cy, scale);
        drawHelicopter(ctx, sim, cx, cy, scale);
    }

    // Wind arrow (draw last, on top)
    if (simulations.length > 0 && simulations[0].results) {
        drawWindArrow(ctx, simulations[0], width, height);
    }
}

function drawCompassRose(ctx, cx, cy, radius) {
    const labels = ['N', 'E', 'S', 'W'];
    const angles = [0, 90, 180, 270];
    ctx.font = '12px monospace';
    ctx.fillStyle = 'rgba(255,255,255,0.5)';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';

    for (let i = 0; i < 4; i++) {
        const a = degreesToRadians(angles[i]) - Math.PI / 2;
        const x = cx + Math.cos(a) * (radius + 2);
        const y = cy + Math.sin(a) * (radius + 2);
        ctx.fillText(labels[i], x, y);
    }

    // Tick marks every 30 degrees
    ctx.strokeStyle = 'rgba(255,255,255,0.15)';
    ctx.lineWidth = 1;
    for (let deg = 0; deg < 360; deg += 30) {
        const a = degreesToRadians(deg) - Math.PI / 2;
        const inner = radius - 8;
        const outer = radius - 2;
        ctx.beginPath();
        ctx.moveTo(cx + Math.cos(a) * inner, cy + Math.sin(a) * inner);
        ctx.lineTo(cx + Math.cos(a) * outer, cy + Math.sin(a) * outer);
        ctx.stroke();
    }
}

function drawFlightPath(ctx, sim, cx, cy, scale) {
    const { x, y, torque } = sim.results;
    const n = x.length;
    const isApproach = sim.results.isApproach;

    // Draw path segments colored by torque
    for (let i = 0; i < n - 1; i++) {
        const j = i + 1;
        const t = torque[i];
        ctx.strokeStyle = torqueColor(t, sim.color);
        ctx.lineWidth = 2;

        // Dashed line during approach phase
        if (isApproach && sim.results.phase[i] === 'approach') {
            ctx.setLineDash([4, 4]);
        } else {
            ctx.setLineDash([]);
        }

        ctx.beginPath();
        ctx.moveTo(cx + x[i] * scale, cy - y[i] * scale);
        ctx.lineTo(cx + x[j] * scale, cy - y[j] * scale);
        ctx.stroke();
    }
    ctx.setLineDash([]);

    // For continuous orbit (not approach), draw closing segment
    if (!isApproach && n > 1) {
        ctx.strokeStyle = torqueColor(torque[n - 1], sim.color);
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(cx + x[n - 1] * scale, cy - y[n - 1] * scale);
        ctx.lineTo(cx + x[0] * scale, cy - y[0] * scale);
        ctx.stroke();
    }
}

function drawHelicopter(ctx, sim, cx, cy, scale) {
    const state = sim.getCurrentState();
    if (!state) return;

    const sx = cx + state.x * scale;
    const sy = cy - state.y * scale;

    ctx.save();
    ctx.translate(sx, sy);
    // Ground track heading (nav convention: 0=north, CW positive).
    // Canvas: 0 angle = right/east, CW positive.
    // Convert: canvas_angle = heading - PI/2
    ctx.rotate(state.groundTrackHeading - Math.PI / 2);

    const size = 14;

    // Fuselage (teardrop pointing right = forward)
    ctx.fillStyle = sim.color;
    ctx.beginPath();
    ctx.ellipse(0, 0, size * 0.7, size * 0.3, 0, 0, Math.PI * 2);
    ctx.fill();

    // Nose accent
    ctx.fillStyle = lightenColor(sim.color, 30);
    ctx.beginPath();
    ctx.ellipse(size * 0.35, 0, size * 0.25, size * 0.2, 0, 0, Math.PI * 2);
    ctx.fill();

    // Tail boom
    ctx.strokeStyle = sim.color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(-size * 0.5, 0);
    ctx.lineTo(-size * 1.1, 0);
    ctx.stroke();

    // Tail rotor disc
    ctx.strokeStyle = 'rgba(255,255,255,0.6)';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(-size * 1.1, -size * 0.25);
    ctx.lineTo(-size * 1.1, size * 0.25);
    ctx.stroke();

    // Main rotor disc
    ctx.strokeStyle = 'rgba(255,255,255,0.4)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.arc(0, 0, size * 0.9, 0, Math.PI * 2);
    ctx.stroke();

    // Rotor blades (use blade count from preset)
    const bladeCount = sim.preset.rotor.bladeCount;
    const now = performance.now() / 50; // animate spin
    ctx.strokeStyle = 'rgba(255,255,255,0.7)';
    ctx.lineWidth = 1.5;
    for (let b = 0; b < bladeCount; b++) {
        const angle = now + (b * Math.PI * 2) / bladeCount;
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(Math.cos(angle) * size * 0.9, Math.sin(angle) * size * 0.9);
        ctx.stroke();
    }

    ctx.restore();
}

function drawWindArrow(ctx, sim, width, height) {
    const wind = sim.results.wind;
    const windMag = Math.sqrt(wind.x * wind.x + wind.y * wind.y);
    if (windMag < 0.1) return;

    const arrowX = width - 60;
    const arrowY = 50;
    const arrowLen = 25;

    // Wind direction (the direction wind is blowing TOWARD)
    const angle = Math.atan2(wind.x, wind.y);

    ctx.save();
    ctx.translate(arrowX, arrowY);

    // Background circle
    ctx.fillStyle = 'rgba(0,0,0,0.4)';
    ctx.beginPath();
    ctx.arc(0, 0, 35, 0, Math.PI * 2);
    ctx.fill();

    // Arrow shaft — points in direction wind blows toward.
    // Canvas rotation matrix: x' = -r*sin(θ), y' = r*cos(θ) for point (0,r).
    // To point arrowhead toward (wind.x, -wind.y) on canvas, negate wind.x in atan2.
    ctx.rotate(Math.atan2(-wind.x, -wind.y));
    ctx.strokeStyle = '#00ccff';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(0, -arrowLen);
    ctx.lineTo(0, arrowLen);
    ctx.stroke();

    // Arrowhead
    ctx.fillStyle = '#00ccff';
    ctx.beginPath();
    ctx.moveTo(0, arrowLen);
    ctx.lineTo(-5, arrowLen - 8);
    ctx.lineTo(5, arrowLen - 8);
    ctx.closePath();
    ctx.fill();

    ctx.restore();

    // Label
    ctx.fillStyle = '#00ccff';
    ctx.font = '10px monospace';
    ctx.textAlign = 'center';
    ctx.fillText(`${(windMag / 0.514444).toFixed(0)} kt`, arrowX, arrowY + 45);
}

// Map torque % to color: green < 85, yellow 85-100, red > 100.
// Blends with the simulation's base color.
function torqueColor(torquePct, baseColor) {
    if (torquePct < 70) return baseColor;
    if (torquePct < 85) return lerpColor(baseColor, '#ffaa00', (torquePct - 70) / 15);
    if (torquePct < 100) return lerpColor('#ffaa00', '#ff3333', (torquePct - 85) / 15);
    return '#ff3333';
}

function lerpColor(c1, c2, t) {
    const r1 = parseInt(c1.slice(1, 3), 16), g1 = parseInt(c1.slice(3, 5), 16), b1 = parseInt(c1.slice(5, 7), 16);
    const r2 = parseInt(c2.slice(1, 3), 16), g2 = parseInt(c2.slice(3, 5), 16), b2 = parseInt(c2.slice(5, 7), 16);
    const r = Math.round(r1 + (r2 - r1) * t);
    const g = Math.round(g1 + (g2 - g1) * t);
    const b = Math.round(b1 + (b2 - b1) * t);
    return `rgb(${r},${g},${b})`;
}

function lightenColor(hex, amount) {
    const r = Math.min(255, parseInt(hex.slice(1, 3), 16) + amount);
    const g = Math.min(255, parseInt(hex.slice(3, 5), 16) + amount);
    const b = Math.min(255, parseInt(hex.slice(5, 7), 16) + amount);
    return `rgb(${r},${g},${b})`;
}
