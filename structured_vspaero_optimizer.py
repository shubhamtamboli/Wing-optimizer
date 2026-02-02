
"""
structured_vspaero_optimizer.py

Structure:
1. Imports & Environment Setup
2. Configuration & Inputs (Constants)
3. Utility Functions
4. Core VSPAERO Interface (Simulation Logic)
5. Objective Function (Optimization Logic)
6. Main Execution Loop
"""

# =============================================================================
# 1. Imports & Environment Setup
# =============================================================================
import os
import sys
import math
import csv
import contextlib
import time

# Third-party imports
from tqdm import tqdm
from platypus import Problem, Real, SMPSO, PM

# OpenVSP Import
# Note: Ensure the environment has OpenVSP installed or paths are set correctly.
try:
    import openvsp as vsp #type: ignore
except ImportError:
    print("Error: Could not import 'openvsp'. Please ensure it is installed in your Python environment.")
    sys.exit(1)


# =============================================================================
# 2. Configuration & Inputs
# =============================================================================

import json

CONFIG_FILE = "optimizer_config.json"

def load_config(config_path=CONFIG_FILE):
    if not os.path.exists(config_path):
        print(f"Error: Config file '{config_path}' not found.")
        sys.exit(1)
    with open(config_path, 'r') as f:
        return json.load(f)

CONFIG = load_config()

# --- Paths ---
# VSPAERO Discovery Logic
def find_vspaero_path():
    # 0. Check Config First
    cfg_path = CONFIG["environment"].get("vsp_path", "AUTO")
    if cfg_path != "AUTO" and os.path.exists(cfg_path):
        return cfg_path

    # 1. Check if 'vspaero' is in the openvsp package directory
    if 'openvsp' in sys.modules:
        import openvsp #type: ignore
        pkg_dir = os.path.dirname(openvsp.__file__)
        possible_loc = os.path.join(pkg_dir, "vspaero.exe" if os.name == 'nt' else "vspaero")
        if os.path.exists(possible_loc):
            print(f"Found VSPAERO in package: {pkg_dir}")
            return pkg_dir

    # 2. Check System PATH
    import shutil
    cmd = "vspaero.exe" if os.name == 'nt' else "vspaero"
    path_loc = shutil.which(cmd)
    if path_loc:
        print(f"Found VSPAERO in PATH: {os.path.dirname(path_loc)}")
        return os.path.dirname(path_loc)
    
    return None

VSP_PATH = find_vspaero_path()
if VSP_PATH:
    print(f"Using VSPAERO Path: {VSP_PATH}")
else:
    print("WARNING: VSPAERO path not found.")


# Output Files
AERO_RESULTS_FILE = CONFIG["environment"]["aero_results_file"]
OPT_RESULTS_FILE = CONFIG["environment"]["opt_results_file"]
AIRFOIL_FILE = CONFIG["environment"]["airfoil_file"]
DEBUG_VSP_FILE = CONFIG["environment"]["debug_vsp_file"]
POLAR_FILE = CONFIG["environment"]["polar_file"]

# --- Aircraft Constants ---
WEIGHT_KG = CONFIG["aircraft"]["weight_kg"]
WING_LOADING_MIN = CONFIG["aircraft"]["wing_loading_min"]
WING_LOADING_MAX = CONFIG["aircraft"]["wing_loading_max"]

# --- Aerodynamic Simulation Parameters ---
RHO = CONFIG["simulation"]["rho_kg_m3"]
VELOCITY = CONFIG["simulation"]["velocity_m_s"]
VISCOSITY = CONFIG["simulation"]["viscosity"]
MACH = CONFIG["simulation"]["mach"]
AOA = CONFIG["simulation"]["aoa_deg"]
BETA = CONFIG["simulation"]["beta_deg"]

# --- Optimization Bounds (Design Variables) ---
dv = CONFIG["design_variables"]

# Calculate Area bounds from wing loading if not explicit (or use constants)
# The logic uses weight/loading to constrain area
# NOTE: We calculate SEMI-AREA limits.
AREA_MIN = WEIGHT_KG / (2 * WING_LOADING_MAX)
AREA_MAX = WEIGHT_KG / (2 * WING_LOADING_MIN)

BOUNDS = {
    "semi_span": (dv["semi_span"]["min"], dv["semi_span"]["max"]),
    "semi_area": (AREA_MIN, AREA_MAX), 
    "taper": (dv["taper"]["min"], dv["taper"]["max"]),
    "sweep": (dv["sweep"]["min"], dv["sweep"]["max"]),
    "twist": (dv["twist"]["min"], dv["twist"]["max"]),
    "root_inc": (dv["root_incidence"]["min"], dv["root_incidence"]["max"])
}

# --- Optimization Settings ---
SWARM_SIZE = CONFIG["optimization"]["swarm_size"]
MAX_EVALUATIONS = CONFIG["optimization"]["max_evaluations"]
STEP_SIZE = CONFIG["optimization"]["step_size"]


# =============================================================================
# 3. Utility Functions
# =============================================================================
# ... (Utilities unchanged) ... 
# (Wait, I need to preserve lines I am not viewing? Better to target specific blocks or replace functions)
# I will cancel this Replace and do separate targeted edits to avoid overwriting unrelated code accidentally.
# I'll use separate calls for separate sections.




# =============================================================================
# 3. Utility Functions
# =============================================================================

@contextlib.contextmanager
def suppress_output_fd():
    """
    Context manager to suppress ALL output from Python and C-level extensions (like OpenVSP).
    Redirects stdout (1) and stderr (2) to os.devnull.
    """
    with open(os.devnull, 'w') as devnull:
        # Save original file descriptors
        old_stdout_fd = os.dup(1)
        old_stderr_fd = os.dup(2)
        try:
            # Redirect to devnull
            os.dup2(devnull.fileno(), 1)
            os.dup2(devnull.fileno(), 2)
            yield
        finally:
            # Restore file descriptors
            os.dup2(old_stdout_fd, 1)
            os.dup2(old_stderr_fd, 2)
            os.close(old_stdout_fd)
            os.close(old_stderr_fd)


def parse_polar_file(filepath):
    """
    Parses a VSPAERO .polar output file.
    Returns a list of dictionaries, where each dict represents a line of results.
    """
    if not os.path.exists(filepath):
        return []

    with open(filepath, 'r') as f:
        lines = f.readlines()
    
    # Find the header line (starts with 'Beta' usually in VSPAERO polars)
    header_idx = None
    for i, line in enumerate(lines):
        if line.strip().startswith("Beta"):
            header_idx = i
            break
            
    if header_idx is None:
        # Could log warning here
        return []

    headers = lines[header_idx].split()
    col_idx = {name: idx for idx, name in enumerate(headers)}
    results = []
    
    # Parse data lines
    for line in lines[header_idx + 1:]:
        cols = line.split()
        if len(cols) != len(headers):
            continue
        try:
            # Convert all columns to floats
            row = {name: float(cols[idx]) for name, idx in col_idx.items()}
            results.append(row)
        except ValueError:
            continue
            
    return results


# Global counter for VSPAERO calls
_vspaero_call_count = 0
_progress_bar = None

def init_environment():
    """Sets up VSP path and initializes output files."""
    # Set VSPAERO Path
    vsp.SetVSPAEROPath(VSP_PATH)
    
    # Initialize Detail Results CSV
    with open(AERO_RESULTS_FILE, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Iteration", "Cl", "Cd", "Cm", "L/D"])


# =============================================================================
# 4. Core VSPAERO Interface
# =============================================================================

def run_vspaero_analysis(semi_span, semi_area, taper, sweep, twist, root_inc):
    """
    Constructs the wing geometry in OpenVSP, meshes it, runs VSPAERO, 
    and returns aerodynamic coefficients.
    """
    global _vspaero_call_count, _progress_bar
    _vspaero_call_count += 1

    # 1. Clear and Build Geometry
    vsp.ClearVSPModel()
    wing_id = vsp.AddGeom("WING", "") # Standard Wing

    # Set Driver Group (Drive by Span, Area, Taper)
    section_idx = 1
    vsp.SetDriverGroup(wing_id, section_idx, vsp.SPAN_WSECT_DRIVER, vsp.AREA_WSECT_DRIVER, vsp.TAPER_WSECT_DRIVER)
    vsp.Update()
    
    # Set Wing Parameters
    # NOTE: OpenVSP "Span" and "Area" parameters act on the Section.
    # For a symmetric wing, these usually map to the SEMI-values if modeling half
    # or the FULL values if modeling full.
    # However, standard practice and our bounds confirm we are setting the half-wing geometry directly.
    vsp.SetParmVal(wing_id, "Span", "XSec_1", semi_span)
    vsp.SetParmVal(wing_id, "Area", "XSec_1", semi_area)
    vsp.SetParmVal(wing_id, "Taper", "XSec_1", taper)
    vsp.SetParmVal(wing_id, "Sweep", "XSec_1", sweep)
    vsp.SetParmVal(wing_id, "Twist", "XSec_1", twist)
    vsp.SetParmVal(wing_id, "Twist", "XSec_0", root_inc)
    
    # Set Tessellation (Resolution)
    vsp.SetParmVal(wing_id, "SectTess_U", "XSec_1", 10) # Spanwise
    vsp.SetParmVal(wing_id, "Tess_W", "Shape", 20)      # Chordwise
    vsp.SetParmVal(wing_id, "LECluster", "WingGeom", 0.2)
    vsp.SetParmVal(wing_id, "TECluster", "WingGeom", 0.2)
    vsp.Update()
    
    # Calculate Reynolds Number based on Mean Aerodynamic Chord (MAC)
    # Using Global Physics Constants
    MAC = vsp.GetParmVal(wing_id, "MAC", "WingGeom")
    reynolds = (RHO * VELOCITY * MAC) / VISCOSITY
    
    # 2. Set Airfoil Sections (ClarkY)
    xsec_surf_id = vsp.GetXSecSurf(wing_id, 0)
    
    # Change type to FILE_AIRFOIL
    vsp.ChangeXSecShape(xsec_surf_id, 0, vsp.XS_FILE_AIRFOIL)
    vsp.ChangeXSecShape(xsec_surf_id, 1, vsp.XS_FILE_AIRFOIL)
    
    root_xsec = vsp.GetXSec(xsec_surf_id, 0)
    tip_xsec = vsp.GetXSec(xsec_surf_id, 1)
    
    # Read Airfoil File
    # Ensure ClarkY.dat is in the same directory!
    vsp.ReadFileAirfoil(root_xsec, AIRFOIL_FILE)
    vsp.ReadFileAirfoil(tip_xsec, AIRFOIL_FILE)
    
    vsp.Update()
    vsp.WriteVSPFile(DEBUG_VSP_FILE)

    # 3. Setup and Run VSPAERO
    
    # Silence Output
    with suppress_output_fd():
        # -- Step A: Geometry/Mesh --
        mesh_analysis = "VSPAEROComputeGeometry"
        vsp.SetAnalysisInputDefaults(mesh_analysis)
        vsp.SetIntAnalysisInput(mesh_analysis, "Symmetry", [2]) # X-Z Symmetry
        vsp.ExecAnalysis(mesh_analysis)
        
        # -- Step B: Aerodynamic Sweep --
        analysis_name = "VSPAEROSweep"
        vsp.SetAnalysisInputDefaults(analysis_name)
        
        # Conditions
        vsp.SetIntAnalysisInput(analysis_name, "Symmetry", [2])
        vsp.SetIntAnalysisInput(analysis_name, "NCPU", [4])
        vsp.SetIntAnalysisInput(analysis_name, "GeomSet", [0]) # 0 = Panel Method
        vsp.SetDoubleAnalysisInput(analysis_name, "Rho", [RHO])
        
        # Ref Area/Span
        vsp.SetDoubleAnalysisInput(analysis_name, "Sref", [2 * semi_area])
        vsp.SetDoubleAnalysisInput(analysis_name, "bref", [2 * semi_span])
        vsp.SetDoubleAnalysisInput(analysis_name, "cref", [MAC])
        
        # Flight Conditions
        vsp.SetDoubleAnalysisInput(analysis_name, "AlphaStart", [AOA])
        vsp.SetDoubleAnalysisInput(analysis_name, "AlphaEnd", [AOA])
        vsp.SetIntAnalysisInput(analysis_name, "AlphaNpts", [1])
        
        vsp.SetDoubleAnalysisInput(analysis_name, "BetaStart", [BETA])
        vsp.SetDoubleAnalysisInput(analysis_name, "BetaEnd", [BETA])
        vsp.SetIntAnalysisInput(analysis_name, "BetaNpts", [1])
        
        vsp.SetDoubleAnalysisInput(analysis_name, "MachStart", [MACH])
        vsp.SetDoubleAnalysisInput(analysis_name, "MachEnd", [MACH])
        vsp.SetIntAnalysisInput(analysis_name, "MachNpts", [1])
        
        vsp.SetDoubleAnalysisInput(analysis_name, "ReCref", [reynolds])
        vsp.SetDoubleAnalysisInput(analysis_name, "ReCrefEnd", [reynolds])
        vsp.SetIntAnalysisInput(analysis_name, "ReCrefNpts", [1])
        
        # Execute
        vsp.ExecAnalysis(analysis_name)

    # 4. Parse Results
    cl, cd, cm, ld = 0.0, 0.0, 0.0, 0.0
    
    polar_results = parse_polar_file(POLAR_FILE)
    if polar_results:
        res = polar_results[0]
        cl = res.get("CLtot", 0.0)
        cd = res.get("CDtot", 0.0)
        cm = res.get("CMytot", 0.0)
        ld = res.get("L/D", 0.0)
        
        # Log this run
        with open(AERO_RESULTS_FILE, mode="a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([_vspaero_call_count, cl, cd, cm, ld])
    else:
        # If parsing failed, return 0 (Optimizer will penalize this via checks)
        print(f"Warning: Parsing failed for iteration {_vspaero_call_count}")

    # Update TQDM
    if _progress_bar is not None:
        _progress_bar.update(1)

    return cl, cd, cm, ld


# =============================================================================
# 5. Objective Function (Platypus Interface)
# =============================================================================

def weighted_sum_wing_analysis(vars):
    """
    Evaluator function for Platypus optimizer.
    input: vars (list) -> [semi_span, semi_area, taper, sweep, twist, root_inc]
    output: ([objectives], [constraints])
    """
    semi_span, semi_area, taper, sweep, twist, root_inc = vars
    
    # Run Physics
    cl, cd, cm, ld = run_vspaero_analysis(semi_span, semi_area, taper, sweep, twist, root_inc)
    
    # Constraints
    # 1. Pitching Moment (Cm) <= 0 (Stable)
    # 2. Lift Coefficient (Cl) >= 0.2
    # 3. Lift Coefficient (Cl) <= 0.25
    c1 = cm
    c2 = -cl + 0.2
    c3 = cl - 0.25
    
    # Objective: Maximize Endurance (Cl^1.5 / Cd)
    # Since Platypus minimizes by default, we return negative.
    if cl <= 0 or cd <= 0:
        endurance = 0.0
    else:
        endurance = -(cl**1.5) / cd
        
    return [endurance], [c1, c2, c3]


# =============================================================================
# 6. Main Execution Loop
# =============================================================================

def main():
    global _progress_bar
    
    # 1. Initialize
    init_environment()
    print("Starting Wing Optimization...")
    print(f"Design Variables: {list(BOUNDS.keys())}")
    
    # 2. Define Problem
    # 6 decision variables, 1 objective, 3 constraints
    problem = Problem(6, 1, 3)
    
    # Set variable bounds
    problem.types[:] = [
        Real(*BOUNDS["semi_span"]),
        Real(*BOUNDS["semi_area"]),
        Real(*BOUNDS["taper"]),
        Real(*BOUNDS["sweep"]),
        Real(*BOUNDS["twist"]),
        Real(*BOUNDS["root_inc"]),
    ]
    
    # Set constraints direction ("<=0")
    problem.constraints[:] = ["<=0", "<=0", "<=0"]
    
    # Set function
    problem.function = weighted_sum_wing_analysis
    
    # 3. Configure Algorithm (SMPSO)
    algorithm = SMPSO(
        problem, 
        swarm_size=SWARM_SIZE,
        leader_size=SWARM_SIZE,
        mutation=PM(probability=1.0/6.0, distribution_index=5)
    )
    
    # 4. Run Optimization
    # Initialize main output file
    with open(OPT_RESULTS_FILE, mode="w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["iteration", "objective", "semi_span", "semi_area", "taper", "sweep", "twist", "Root_inc"])
        
        print(f"Running for {MAX_EVALUATIONS} evaluations with swarm size {SWARM_SIZE}...")
        _progress_bar = tqdm(total=SWARM_SIZE * (MAX_EVALUATIONS / STEP_SIZE), desc="VSPAERO Runs", unit="run")
        
        try:
            for i in range(0, MAX_EVALUATIONS, STEP_SIZE):
                algorithm.run(STEP_SIZE)
                
                # Check for feasible solutions
                feasible_sols = [s for s in algorithm.result if getattr(s, "feasible", False)]
                
                if feasible_sols:
                    best = min(feasible_sols, key=lambda s: s.objectives[0])
                    # Write to CSV
                    writer.writerow([
                        i + STEP_SIZE,
                        best.objectives[0],
                        *[f"{v:.6f}" for v in best.variables]
                    ])
                    # Force flush to see results immediately
                    csvfile.flush()
                    
        except KeyboardInterrupt:
            print("\nOptimization interrupted by user.")
        finally:
            _progress_bar.close()

    # 5. Report Final Results
    print("\nOptimization Complete.")
    
    # Find best feasible in final population
    feasible_sols = [s for s in algorithm.result if getattr(s, "feasible", False)]
    if feasible_sols:
        best_sol = min(feasible_sols, key=lambda s: s.objectives[0])
        best_params = best_sol.variables
        
        print("\n=== Best Feasible Solution Found ===")
        print(f"Objective (Negative Endurance): {best_sol.objectives[0]:.6f}")
        print("-" * 30)
        print(f"Semi-Span      : {best_params[0]:.4f} m (Full Span: {2*best_params[0]:.4f} m)")
        print(f"Semi-Area      : {best_params[1]:.4f} m^2 (Full Area: {2*best_params[1]:.4f} m^2)")
        print(f"Taper Ratio    : {best_params[2]:.4f}")
        print(f"Sweep          : {best_params[3]:.4f} deg")
        print(f"Twist          : {best_params[4]:.4f} deg")
        print(f"Root Incidence : {best_params[5]:.4f} deg")
    else:
        print("\nNo feasible solution found satisfying all constraints.")

if __name__ == "__main__":
    main()
