# VSPAERO Wing Optimizer

This repository contains a Python-based aerodynamic optimization tool for wing geometry using **OpenVSP** and **VSPAERO**. The script `structured_vspaero_optimizer.py` automates the process of generating wing geometries, running aerodynamic analysis, and optimizing design variables to maximize endurance.

## Features

- **Automated Geometry Generation**: dynamic creation of wing shapes in OpenVSP.
- **Aerodynamic Analysis**: Integrated VSPAERO simulation for Lift ($C_L$), Drag ($C_D$), and Moment ($C_m$) coefficients.
- **Optimization**: Uses the **Platypus** library (SMPSO algorithm) for multi-objective optimization (though configured here for single-objective with constraints).
- **Configurable**: All parameters (aircraft weight, flight conditions, optimization bounds) are defined in `optimizer_config.json`.

## Requirements

- **Python 3.x**
- **OpenVSP**: You must have OpenVSP installed. The script attempts to auto-discover `vsp` and `vspaero`, or you can set the path in the config.
- **Python Libraries**:
  - `openvsp` (Python API for OpenVSP)
  - `platypus-opt` (Optimization framework)
  - `tqdm` (Progress bar)

### Installation

Install the required Python packages:

```bash
pip install platypus-opt tqdm
```

Ensure the `openvsp` Python module is accessible. This usually involves adding the OpenVSP python directory to your `PYTHONPATH` or installing it via an installer if available.

## Configuration

The optimization is controlled by `optimizer_config.json`. Key sections include:

- **Environment**: File paths for results and the input airfoil (`ClarkY.dat`).
- **Aircraft**: Design constraints like Weight and Wing Loading limits.
- **Simulation**: Flight conditions (Velocity, Density, Viscosity, Mach, Alpha, Beta).
- **Design Variables**: Bounds for Semi-Span, Taper, Sweep, Twist, and Root Incidence.
- **Optimization**: Swarm size and maximum evaluations.

**Example Config Snippet:**

```json
"design_variables": {
    "semi_span": { "min": 0.5, "max": 0.75 },
    "taper": { "min": 0.5, "max": 1.0 }
}
```

## Usage

1.  Place `structured_vspaero_optimizer.py`, `optimizer_config.json`, and your airfoil file (e.g., `ClarkY.dat`) in the same directory.
2.  Run the script:

```bash
python structured_vspaero_optimizer.py
```

3.  **Progress**: The script will show a progress bar for the VSPAERO simulations.
4.  **Results**:
    - `wingopt_results.csv`: Log of optimization iterations and the best solution found.
    - `aero_results.csv`: detailed log of every aerodynamic evaluation.
    - `modified_wing.vsp3`: The OpenVSP model of the last evaluated wing.

## Optimization Goal

- **Objective**: Maximize Endurance Parameter ($C_L^{1.5} / C_D$)
- **Constraints**:
  - Pitching Moment coefficient ($C_m$) $\le 0$ (Stable)
  - Lift Coefficient ($C_L$): $0.2 \le C_L \le 0.25$

## Troubleshooting

- **VSPAERO not found**: Ensure OpenVSP is installed and the path is correctly set in `optimizer_config.json` under `vsp_path` if auto-discovery fails.
- **ImportError: No module named 'openvsp'**: Add the OpenVSP python directory to your system's `PYTHONPATH`.
