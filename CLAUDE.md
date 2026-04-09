# Helicopter Turn Simulator

A browser-based helicopter coordinated turn simulator built with vanilla HTML, CSS, and JavaScript. It visualizes a helicopter flying a 360-degree circular turn on an HTML canvas while computing aerodynamic metrics in real time.

## Purpose

This project demonstrates the critical difference between a helicopter pivoting in a circle through the air mass versus performing a turn around a fixed ground point. When wind is present, maintaining a ground-referenced circular path requires constantly varying airspeed, bank angle adjustments, and — most importantly — significantly more power than a no-wind orbit. The simulator makes this increased power demand visible and quantifiable.

## How It Works

The user provides flight parameters (helicopter weight, entry heading, wind direction/speed, desired turn radius, and ground speed). The simulation engine then:

1. Computes airspeed at each point in the turn by combining ground speed with wind vectors
2. Calculates the required bank angle and load factor for the turn radius
3. Breaks total power into induced, profile, parasite, and maneuvering components using simplified rotor aerodynamics
4. Animates the helicopter tracing the flight path on a canvas with a live readout of bank angle and power required

## Structure

- `public/index.html` — Main page with simulator controls and canvas
- `public/script.js` — Simulation engine, aerodynamic calculations, and canvas animation
- `public/styles.css` — UI styling
- `.gitlab-ci.yml` — GitLab Pages deployment config
