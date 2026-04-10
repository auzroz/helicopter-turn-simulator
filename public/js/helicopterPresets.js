// Helicopter type presets with realistic parameters sourced from
// POH / type certificate data. All SI units unless noted.

const PRESETS = [
    {
        id: 'r22',
        name: 'Robinson R22',
        category: 'Light Piston',
        weight: { empty: 858, maxGross: 1370, typical: 1200 },   // lbs
        rotor: {
            radius: 3.83,          // m
            bladeCount: 2,
            rpm: 510,
            solidity: 0.0315,
            chord: 0.19            // m
        },
        airframe: {
            flatPlateArea: 0.6,    // m^2
            bladeProfileDragCoeff: 0.011,
            inducedPowerFactor: 1.15
        },
        performance: {
            maxContinuousPower: 95,    // kW shaft power
            vne: 102,                  // knots
            typicalCruise: 75          // knots
        }
    },
    {
        id: 'r44',
        name: 'Robinson R44',
        category: 'Light Piston',
        weight: { empty: 1500, maxGross: 2500, typical: 2200 },
        rotor: {
            radius: 5.03,
            bladeCount: 2,
            rpm: 408,
            solidity: 0.0335,
            chord: 0.265
        },
        airframe: {
            flatPlateArea: 0.9,
            bladeProfileDragCoeff: 0.011,
            inducedPowerFactor: 1.15
        },
        performance: {
            maxContinuousPower: 183,
            vne: 120,
            typicalCruise: 100
        }
    },
    {
        id: 'bell206',
        name: 'Bell 206 JetRanger',
        category: 'Light Turbine',
        weight: { empty: 1830, maxGross: 3200, typical: 2900 },
        rotor: {
            radius: 5.08,
            bladeCount: 2,
            rpm: 394,
            solidity: 0.0450,
            chord: 0.33
        },
        airframe: {
            flatPlateArea: 1.1,
            bladeProfileDragCoeff: 0.012,
            inducedPowerFactor: 1.15
        },
        performance: {
            maxContinuousPower: 280,
            vne: 130,
            typicalCruise: 110
        }
    },
    {
        id: 'bell407',
        name: 'Bell 407',
        category: 'Medium Turbine',
        weight: { empty: 2722, maxGross: 5250, typical: 4500 },
        rotor: {
            radius: 5.33,
            bladeCount: 4,
            rpm: 413,
            solidity: 0.0654,
            chord: 0.273
        },
        airframe: {
            flatPlateArea: 1.3,
            bladeProfileDragCoeff: 0.012,
            inducedPowerFactor: 1.15
        },
        performance: {
            maxContinuousPower: 510,
            vne: 140,
            typicalCruise: 120
        }
    },
    {
        id: 'as350',
        name: 'Airbus AS350 / H125',
        category: 'Medium Turbine',
        weight: { empty: 2756, maxGross: 5512, typical: 4300 },
        rotor: {
            radius: 5.35,
            bladeCount: 3,
            rpm: 394,
            solidity: 0.0550,
            chord: 0.305
        },
        airframe: {
            flatPlateArea: 1.2,
            bladeProfileDragCoeff: 0.012,
            inducedPowerFactor: 1.15
        },
        performance: {
            maxContinuousPower: 540,
            vne: 155,
            typicalCruise: 120
        }
    },
    {
        id: 'uh60',
        name: 'UH-60 Black Hawk',
        category: 'Heavy Turbine',
        weight: { empty: 11516, maxGross: 22000, typical: 16000 },
        rotor: {
            radius: 8.18,
            bladeCount: 4,
            rpm: 258,
            solidity: 0.0826,
            chord: 0.527
        },
        airframe: {
            flatPlateArea: 3.5,
            bladeProfileDragCoeff: 0.012,
            inducedPowerFactor: 1.15
        },
        performance: {
            maxContinuousPower: 2230,
            vne: 193,
            typicalCruise: 150
        }
    }
];

// Derive angular speed in rad/s from RPM for each preset
for (const p of PRESETS) {
    p.rotor.omega = (p.rotor.rpm * 2 * Math.PI) / 60;
}

export function getPreset(id) {
    return PRESETS.find(p => p.id === id);
}

export function getAllPresets() {
    return PRESETS;
}
