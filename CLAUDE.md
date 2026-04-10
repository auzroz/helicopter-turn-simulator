# Helicopter Turn Simulator

A browser-based helicopter coordinated turn simulator built with vanilla HTML, CSS, and JavaScript (ES modules). It visualizes a helicopter flying a 360-degree circular turn on an HTML canvas while computing aerodynamic metrics in real time.

## Purpose

This project demonstrates the critical difference between a helicopter pivoting in a circle through the air mass versus performing a turn around a fixed ground point. When wind is present, maintaining a ground-referenced circular path requires constantly varying airspeed, bank angle adjustments, and — most importantly — significantly more power than a no-wind orbit. The simulator makes this increased power demand visible and quantifiable.

## How It Works

The user selects a helicopter type (R22, R44, Bell 206, Bell 407, AS350/H125, UH-60) and flight parameters (entry heading, wind direction/speed, desired turn radius, and ground speed). The simulation engine then:

1. Computes airspeed at each point in the turn by combining ground speed with wind vectors
2. Calculates the required bank angle and load factor for the turn radius
3. Breaks total power into induced, profile, parasite, and maneuvering components using simplified rotor aerodynamics
4. Displays torque as a percentage of the helicopter's max continuous power — the key metric pilots monitor
5. Animates the helicopter tracing the flight path on a canvas with live instrument gauges

Up to 4 simulations can run simultaneously for comparison (e.g., different wind conditions or helicopter types).

## Structure

- `public/index.html` — Main page layout (setup bar, sidebar, canvas)
- `public/styles.css` — Dark aviation-themed UI styling
- `public/js/main.js` — Entry point, UI event wiring
- `public/js/helicopterPresets.js` — Realistic parameters for 6 helicopter types
- `public/js/physics.js` — Aerodynamic calculations (pure functions)
- `public/js/simulation.js` — SimulationInstance class (state + stepping)
- `public/js/simulationManager.js` — Multi-sim manager, requestAnimationFrame loop
- `public/js/rendererCanvas.js` — Flight path + helicopter silhouette rendering
- `public/js/rendererGauges.js` — Circular dial gauges (bank, torque, airspeed)
- `public/js/utils.js` — Unit conversions, math helpers
- `.gitlab-ci.yml` — GitLab Pages deployment config
