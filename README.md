# Multi-CUAV Spectrum Sharing — professor review package

This is a *path-isolated modular extraction* of the six user-provided simulation scripts. The underlying mathematical optimization routines have **not been rederived or numerically validated here**. The earlier original full scripts are preserved under `Original_Scripts/` for comparison.

## Launch

1. Install CVX 2.2 and run `cvx_setup` once; CVX 2.2 with SDPT3 was already confirmed available on the user's machine at `C:\cvx`.
2. From a **fresh MATLAB session**, set current directory to this repository's root.
3. Open a chosen script from `Main_Simulations/`, and press Run. The script adds the project root and configures **only the function folders required for that experiment**, without removing CVX from the MATLAB path. When MATLAB asks for current folder, choose **Change Folder**.
4. Use NDrops=1, MC=2 as a smoke test where applicable; restore the recorded production settings before comparing with figures in the paper.
5. Check for errors/warnings and verify that optimized trajectories actually changed and protection constraints remained satisfied. A printed sum rate does **not** prove convergence.

## Folder layout

- `Main_Simulations`: six entry-point scripts, approx. 90–300 lines each (convergence reduced from 4,799 to ~140 lines by removing thousands of old commented lines).
- `Common_Functions`: reusable MATLAB implementations for most main experiments.
- `Convergence_Overrides`: special trajectory implementations needed for convergence history; these have a different output signature than their Common counterparts.
- `Cases_Functions`: specialized interference-case routines.
- `Unlicensed_Power_Functions`: an independent implementation with its own function names; it is intentionally isolated from Common_Functions.
- `Original_Scripts`: source snapshots for independent verification.
- `configure_simulation_paths.m`: isolates the selected function directories; preserves CVX.

## Known unresolved runtime issue

In the user's MATLAB session, `joint_traj_cvx_analytic.m` generated a CVX error while declaring X. The historical code catches errors and returns the previous trajectory. Thus a run may still print sum rates even if trajectory optimization failed. This package improves isolation and error reporting, but it **does not claim this CVX error is fixed**. Independently test and compare all six experiments before paper submission. Do not use results from runs with CVX trajectory warnings as proof of successful trajectory optimization.

## Reproducibility note

This environment does not contain MATLAB/CVX; syntax/execution and expected IEEE paper figures have **not** been independently reproduced. Document exact seed, solver, flight time, NDrops, MC and any additional toolbox requirements during final verification.
