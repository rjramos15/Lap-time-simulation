## Repository Structure

This repository is organised into two main directories:

### 1. `Plots`
This folder contains post-processed results and visualization scripts.

- `Direct Drive`, `Two Stage Gearbox`, and `Three Stage Gearbox`  
  These subfolders contain the graphical outputs corresponding to the ideal cases for each drivetrain configuration.

- MATLAB scripts  
  Two scripts are provided:
  - one for visualising the mesh convergence study
  - one for comparing results across different model configurations

### 2. `Lap time Simulation (gearbox)`
This is the main simulation directory.

It contains all the required files to run the lap time simulations for:
- single-speed (direct drive)
- two-speed gearbox
- three-speed gearbox

To run a simulation, open the corresponding MATLAB script and ensure all files in this directory are included in the working path.

---

## How to use
1. Run the simulation scripts inside `Lap time Simulation (gearbox)` to generate results.
2. Use the scripts in `plots` to reproduce convergence studies and comparison figures.
3. Refer to the subfolders in `plots` for pre-generated results of the ideal configurations.

## License

This project is protected under a custom license.
See the LICENSE file for details.
