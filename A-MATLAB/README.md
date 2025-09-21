## System identification (SINDYc)

A parsimonious, interpretable model of the MIRTE Master is identified using **SINDYc**.  
The MATLAB implementation and scripts are provided in `A - MATLAB/`.

---

## Robust trajectory tracking (Tube NMPC)

A **tube-based NMPC** design provides robustness to model mismatch and disturbances via a two-layer structure:

1. **Nominal layer (NMPC):** Uses the SINDYc-identified model to compute a nominal trajectory along the reference path.
2. **Ancillary layer (DLQR):** Keeps the real system inside a **robust positively invariant (RPI)** tube around the nominal trajectory.

The workflow:
- Prototype in MATLAB (`A - MATLAB/`).
- Run offline simulations with **acados** in `B - Tube-NMPC Offline/`.
- Deploy on the MIRTE Master robot from `C - Tube-NMPC Robot Deployment/`.
