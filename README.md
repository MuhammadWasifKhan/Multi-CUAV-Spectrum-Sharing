# Multi-CUAV Spectrum Sharing — MATLAB Implementation of AI-IBPC and IA-RSS

This repository provides modular MATLAB implementations of the proposed AI-IBPC and IA-RSS schemes for multi-CUAV spectrum sharing over licensed and unlicensed bands. The implementations have been tested using MATLAB and CVX.

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
- [Original MATLAB implementations](https://github.com/MuhammadWasifKhan/Multi-CUAV-Spectrum-Sharing/tree/main): standalone MATLAB scripts retained in the `main` branch for reference.
- `configure_simulation_paths.m`: isolates the selected function directories; preserves CVX.

## Reproducibility note

The MATLAB implementations have been tested using CVX.
The simulation parameters, random seeds, and solver
settings should be maintained as specified in the
corresponding scripts when reproducing the numerical results.

