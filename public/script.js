// Retrieve elements
const weightInput = document.getElementById('weight');
const headingInput = document.getElementById('heading');
const windDirectionInput = document.getElementById('wind-direction');
const windSpeedInput = document.getElementById('wind-speed');
const turnRadiusInput = document.getElementById('turn-radius');
const groundSpeedInput = document.getElementById('ground-speed');
const startButton = document.getElementById('start-simulation');

const canvas = document.getElementById('simulation-canvas');
const ctx = canvas.getContext('2d');

const bankAngleDisplay = document.getElementById('bank-angle');
const powerRequiredDisplay = document.getElementById('power-required');

// Constants (you can adjust these based on your helicopter model)
const R = 10;           // Rotor radius in meters
const Omega = 30;       // Rotor angular speed in rad/s
const rho = 1.225;      // Air density in kg/m^3
const sigma = 0.07;     // Rotor solidity
const C_D0 = 0.012;     // Blade profile drag coefficient
const k = 1.15;         // Induced power factor
const S_eq = 1.5;       // Equivalent flat-plate area in m^2
const g = 9.81;         // Gravitational acceleration in m/s^2

startButton.addEventListener('click', startSimulation);

function startSimulation() {
    // Retrieve input values
    const weightLbs = parseFloat(weightInput.value);
    const thetaEntry = parseFloat(headingInput.value);
    const phiWind = parseFloat(windDirectionInput.value);
    const windSpeedKnots = parseFloat(windSpeedInput.value);
    const Rc = parseFloat(turnRadiusInput.value);
    const groundSpeedKnots = parseFloat(groundSpeedInput.value);

    // Convert weight from pounds to Newtons (1 pound ≈ 4.44822 N)
    const W = weightLbs * 4.44822;  // Weight in Newtons

    // Convert speeds from knots to meters per second (1 knot = 0.514444 m/s)
    const VWind = windSpeedKnots * 0.514444;  // Wind speed in m/s
    const Vg = groundSpeedKnots * 0.514444;   // Ground speed in m/s

    // Convert degrees to radians
    const thetaEntryRad = degreesToRadians(thetaEntry);
    const phiWindRad = degreesToRadians(phiWind);

    // Compute angular rate and time
    const omega = Vg / Rc;
    const totalTime = (2 * Math.PI) / omega;
    const numSteps = 360;
    const dt = totalTime / numSteps;

    // Initialize arrays
    const t = [];
    const psi = [];
    const Va = [];
    const theta = [];
    const bankAngle = [];
    const powerTotal = [];
    const x = [];
    const y = [];

    // Rotor disk area and induced velocity in hover
    const A = Math.PI * R * R;
    const Vi0 = Math.sqrt(W / (2 * rho * A));

    // Simulation loop
    for (let i = 0; i < numSteps; i++) {
        t[i] = i * dt;
        psi[i] = omega * t[i];

        // Wind components
        const Vwx = VWind * Math.cos(phiWindRad);
        const Vwy = VWind * Math.sin(phiWindRad);

        // Ground speed components
        const Vgx = -Vg * Math.sin(psi[i]);
        const Vgy = Vg * Math.cos(psi[i]);

        // Airspeed components
        const Vax = Vgx - Vwx;
        const Vay = Vgy - Vwy;

        // Airspeed magnitude and heading
        Va[i] = Math.sqrt(Vax * Vax + Vay * Vay);
        theta[i] = Math.atan2(Vay, Vax);

        // Bank angle
        bankAngle[i] = Math.atan(Vg * Vg / (g * Rc));

        // Load factor
        const n = 1 / Math.cos(bankAngle[i]);

        // Induced velocity
        const Vi = Vi0 / Math.sqrt(1 + (Va[i] / Vi0) ** 2);

        // Power calculations
        const Pinduced = k * W * Vi;
        const Pprofile = (sigma * C_D0 * (rho * Omega ** 3 * R ** 3)) / 8;
        const Pparasite = 0.5 * rho * Va[i] ** 3 * S_eq;
        const Pmaneuver = (n - 1) * Pinduced;

        powerTotal[i] = Pinduced + Pprofile + Pparasite + Pmaneuver;

        // Positions
        x[i] = Rc * Math.cos(psi[i]);
        y[i] = Rc * Math.sin(psi[i]);
    }

    // Compute maxCoordinate and scale once
    const maxCoordinate = Math.max(...x.map(Math.abs), ...y.map(Math.abs));
    const scale = (canvas.width / 2 - 50) / maxCoordinate;  // Subtract padding

    // Start animation
    animateSimulation(x, y, Va, bankAngle, powerTotal, theta, dt, scale);
}

function animateSimulation(x, y, Va, bankAngle, powerTotal, theta, dt, scale) {
    let i = 0;
    const numSteps = x.length;

    function animate() {
        if (i >= numSteps) i = 0;

        // Clear canvas
        ctx.clearRect(0, 0, canvas.width, canvas.height);

        // Draw flight path
        drawFlightPath(x, y, scale);

        // Draw helicopter at current position
        drawHelicopter(x[i], y[i], theta[i], bankAngle[i], scale);

        // Update data display
        bankAngleDisplay.textContent = (radiansToDegrees(bankAngle[i])).toFixed(2);
        powerRequiredDisplay.textContent = powerTotal[i].toFixed(0);

        i++;
        setTimeout(animate, dt * 1000);  // Convert dt to milliseconds
    }

    animate();
}

function drawFlightPath(x, y, scale) {
    ctx.save();

    // Translate to center
    ctx.translate(canvas.width / 2, canvas.height / 2);
    ctx.scale(scale, -scale);  // Flip Y-axis

    // Draw path
    ctx.beginPath();
    ctx.moveTo(x[0], y[0]);
    for (let i = 1; i < x.length; i++) {
        ctx.lineTo(x[i], y[i]);
    }
    ctx.strokeStyle = '#007bff';
    ctx.lineWidth = 1 / scale;
    ctx.stroke();

    ctx.restore();
}

function drawHelicopter(xPos, yPos, heading, bankAngle, scale) {
    ctx.save();

    // Translate to center
    ctx.translate(canvas.width / 2, canvas.height / 2);
    ctx.scale(scale, -scale);  // Flip Y-axis
    ctx.translate(xPos, yPos);

    // Rotate for heading
    ctx.rotate(-heading);

    // Apply bank angle (roll)
    ctx.transform(1, 0, 0, Math.cos(bankAngle), 0, 0);

    // Adjust helicopter size based on scale
    const helicopterSize = 10 / scale;

    // Draw helicopter body (simple representation)
    ctx.fillStyle = '#ff0000';
    ctx.beginPath();
    ctx.rect(-helicopterSize / 2, -helicopterSize / 4, helicopterSize, helicopterSize / 2);
    ctx.fill();

    // Draw rotor blades
    ctx.strokeStyle = '#000000';
    ctx.beginPath();
    ctx.moveTo(-helicopterSize, 0);
    ctx.lineTo(helicopterSize, 0);
    ctx.stroke();

    ctx.restore();
}

function degreesToRadians(degrees) {
    return degrees * Math.PI / 180;
}

function radiansToDegrees(radians) {
    return radians * 180 / Math.PI;
}
