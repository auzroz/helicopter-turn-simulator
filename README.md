# Helicopter Turn Simulator

A browser-based helicopter coordinated turn simulator that visualizes a helicopter flying a 360-degree circular turn while computing aerodynamic metrics in real time.

## Live Demo

[https://auzroz.github.io/helicopter-turn-simulator/](https://auzroz.github.io/helicopter-turn-simulator/)

## Purpose

This simulator demonstrates the critical difference between a helicopter orbiting through the air mass versus performing a turn around a fixed ground point. When wind is present, maintaining a ground-referenced circular path requires constantly varying airspeed and — most importantly — significantly more power than a no-wind orbit. The simulator makes this increased power demand visible and quantifiable through torque percentage readouts.

## Features

- **6 helicopter presets** — R22, R44, Bell 206 JetRanger, Bell 407, AS350/H125, UH-60 Black Hawk
- **Up to 4 simultaneous simulations** — Compare different helicopters or wind conditions side by side
- **Torque as % of max continuous** — The key metric pilots monitor, shown on a circular gauge
- **Linked parameter chain** — See how radius → bank angle → G-load → airspeed → torque cascade
- **Color-coded flight path** — Green/yellow/red based on torque demand around the turn
- **Animated helicopter silhouette** — Top-down view with spinning rotors matching blade count

## Usage

1. Select a helicopter type from the dropdown
2. Set flight parameters: heading, wind direction/speed, turn radius, and ground speed
3. Click **Add Simulation** to start
4. Add more simulations with different parameters to compare

## Built With

Vanilla HTML, CSS, and JavaScript (ES modules). No frameworks or build tools required.
