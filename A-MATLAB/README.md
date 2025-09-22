## System identification (SINDYc)

A parsimonious, interpretable model of the MIRTE Master is identified using SINDYc.  

Run `SINDYc.m` to identify one of three models:

- Kinematic Model
- Extended Kinematic Model
- Dynamic Model

When the script starts, a prompt asks which model to identify and the resulting equations are printed in the Command Window.
`TVRegDiff.m` is required for total-variation–regularised derivative estimates [1].


The identified systems are validated in `Model_Validation.m` using two trajectories:

- A rectangular path with constant heading
- A rectangular path with changing heading

A prompt asks which trajectory to use when validating the identified models.

---

## Robust trajectory tracking (Tube-based NMPC)

A tube-based NMPC design provides robustness to model mismatch and disturbances via a two-layer structure:

1. **Nominal layer (NMPC):** Uses the SINDYc extended kinematic model to compute a nominal trajectory along the reference path.
2. **Ancillary layer (DLQR):** Keeps the real system inside a robust positive invariant (RPI) set around the nominal trajectory.

`Disturbance_Estimate.m` estimates the maximum disturbance bound from the training data.

`Tube_NMPC.m` implements the tube-based controller and lets the user choose one of three reference trajectories:

- A rectangular path with constant heading
- A circular path with constant heading
- A circular path with changing heading

---
## References:
[1] Rick Chartrand, "Numerical differentiation of noisy, nonsmooth data," ISRN Applied Mathematics, Vol. 2011, Article ID 164564, 2011. 