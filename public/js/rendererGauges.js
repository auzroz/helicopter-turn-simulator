// Renders circular dial gauges on small canvases for each simulation.

const TAU = Math.PI * 2;
const DEG = Math.PI / 180;

// Gauge arc spans 270 degrees (from 225° to -45° i.e. 315°)
const ARC_START = 135 * DEG;  // 7 o'clock position
const ARC_SPAN  = 270 * DEG;

function setupGaugeCanvas(canvas, size) {
    const dpr = window.devicePixelRatio || 1;
    canvas.width = size * dpr;
    canvas.height = size * dpr;
    canvas.style.width = size + 'px';
    canvas.style.height = size + 'px';
    const ctx = canvas.getContext('2d');
    ctx.scale(dpr, dpr);
    return ctx;
}

function drawGaugeBase(ctx, cx, cy, r, label) {
    // Face
    ctx.fillStyle = '#1a1a2e';
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, TAU);
    ctx.fill();

    // Bezel
    ctx.strokeStyle = '#3a3a5e';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, TAU);
    ctx.stroke();

    // Label
    ctx.fillStyle = 'rgba(255,255,255,0.5)';
    ctx.font = '9px monospace';
    ctx.textAlign = 'center';
    ctx.fillText(label, cx, cy + r * 0.55);
}

function valueToAngle(value, min, max) {
    const t = (value - min) / (max - min);
    return ARC_START + t * ARC_SPAN;
}

function drawNeedle(ctx, cx, cy, r, angle, color) {
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(angle);
    // Needle points along +X (canvas 0° = 3 o'clock), matching arc angle convention
    ctx.strokeStyle = color || '#ffffff';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(r * 0.75, 0);
    ctx.stroke();

    // Needle tip
    ctx.fillStyle = color || '#ffffff';
    ctx.beginPath();
    ctx.moveTo(r * 0.75, 0);
    ctx.lineTo(r * 0.65, -2);
    ctx.lineTo(r * 0.65, 2);
    ctx.closePath();
    ctx.fill();

    // Center hub
    ctx.fillStyle = '#666';
    ctx.beginPath();
    ctx.arc(0, 0, 3, 0, TAU);
    ctx.fill();

    ctx.restore();
}

function drawTickMarks(ctx, cx, cy, r, min, max, majorStep, minorStep) {
    ctx.strokeStyle = 'rgba(255,255,255,0.6)';
    ctx.fillStyle = 'rgba(255,255,255,0.7)';
    ctx.font = '8px monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';

    for (let v = min; v <= max; v += minorStep) {
        const angle = valueToAngle(v, min, max);
        const isMajor = Math.abs(v % majorStep) < 0.01 || Math.abs(v % majorStep - majorStep) < 0.01;
        const innerR = isMajor ? r * 0.6 : r * 0.7;
        const outerR = r * 0.78;

        ctx.lineWidth = isMajor ? 1.5 : 0.5;
        ctx.beginPath();
        ctx.moveTo(cx + Math.cos(angle) * innerR, cy + Math.sin(angle) * innerR);
        ctx.lineTo(cx + Math.cos(angle) * outerR, cy + Math.sin(angle) * outerR);
        ctx.stroke();

        if (isMajor) {
            const labelR = r * 0.48;
            ctx.fillText(
                String(Math.round(v)),
                cx + Math.cos(angle) * labelR,
                cy + Math.sin(angle) * labelR
            );
        }
    }
}

function drawColorArc(ctx, cx, cy, r, min, max, ranges) {
    const arcR = r * 0.73;
    ctx.lineWidth = 4;
    for (const { from, to, color } of ranges) {
        const a1 = valueToAngle(Math.max(from, min), min, max);
        const a2 = valueToAngle(Math.min(to, max), min, max);
        ctx.strokeStyle = color;
        ctx.beginPath();
        ctx.arc(cx, cy, arcR, a1, a2);
        ctx.stroke();
    }
}

function drawDigitalReadout(ctx, cx, cy, r, text, color) {
    ctx.fillStyle = color || '#00ff88';
    ctx.font = 'bold 13px monospace';
    ctx.textAlign = 'center';
    ctx.fillText(text, cx, cy + r * 0.3);
}

// --- Public gauge drawing functions ---

export function drawTorqueGauge(canvas, torquePct, simColor) {
    const size = 120;
    const ctx = setupGaugeCanvas(canvas, size);
    const cx = size / 2, cy = size / 2, r = size / 2 - 6;
    const min = 0, max = 120;

    drawGaugeBase(ctx, cx, cy, r, 'TORQUE %');

    drawColorArc(ctx, cx, cy, r, min, max, [
        { from: 0,   to: 85,  color: '#2ecc71' },
        { from: 85,  to: 100, color: '#f1c40f' },
        { from: 100, to: 120, color: '#e74c3c' }
    ]);

    drawTickMarks(ctx, cx, cy, r, min, max, 20, 10);

    const clamped = Math.min(torquePct, max);
    const angle = valueToAngle(clamped, min, max);
    drawNeedle(ctx, cx, cy, r, angle, simColor);

    const readoutColor = torquePct > 100 ? '#e74c3c' : torquePct > 85 ? '#f1c40f' : '#00ff88';
    drawDigitalReadout(ctx, cx, cy, r, `${torquePct.toFixed(1)}%`, readoutColor);
}

export function drawBankGauge(canvas, bankAngleDeg, simColor) {
    const size = 120;
    const ctx = setupGaugeCanvas(canvas, size);
    const cx = size / 2, cy = size / 2, r = size / 2 - 6;
    const min = 0, max = 60;

    drawGaugeBase(ctx, cx, cy, r, 'BANK °');

    drawColorArc(ctx, cx, cy, r, min, max, [
        { from: 0,  to: 30, color: '#2ecc71' },
        { from: 30, to: 45, color: '#f1c40f' },
        { from: 45, to: 60, color: '#e74c3c' }
    ]);

    drawTickMarks(ctx, cx, cy, r, min, max, 15, 5);

    const clamped = Math.min(Math.abs(bankAngleDeg), max);
    const angle = valueToAngle(clamped, min, max);
    drawNeedle(ctx, cx, cy, r, angle, simColor);

    drawDigitalReadout(ctx, cx, cy, r, `${bankAngleDeg.toFixed(1)}°`, '#00ff88');
}

export function drawAirspeedGauge(canvas, airspeedKnots, vne, simColor) {
    const size = 120;
    const ctx = setupGaugeCanvas(canvas, size);
    const cx = size / 2, cy = size / 2, r = size / 2 - 6;
    const min = 0;
    const max = Math.ceil(vne / 20) * 20 + 20; // round up to nice number above Vne

    drawGaugeBase(ctx, cx, cy, r, 'GS kt');

    drawColorArc(ctx, cx, cy, r, min, max, [
        { from: 0,          to: vne * 0.9, color: '#2ecc71' },
        { from: vne * 0.9,  to: vne,       color: '#f1c40f' },
        { from: vne,        to: max,       color: '#e74c3c' }
    ]);

    const majorStep = max <= 120 ? 20 : max <= 200 ? 40 : 50;
    drawTickMarks(ctx, cx, cy, r, min, max, majorStep, majorStep / 4);

    const clamped = Math.min(airspeedKnots, max);
    const angle = valueToAngle(clamped, min, max);
    drawNeedle(ctx, cx, cy, r, angle, simColor);

    drawDigitalReadout(ctx, cx, cy, r, `${airspeedKnots.toFixed(0)}`, '#00ff88');
}
