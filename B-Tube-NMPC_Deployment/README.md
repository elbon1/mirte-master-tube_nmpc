# Tube-NMPC deployment on MIRTE Master

This package provides a real-time, tube-based NMPC controller for the MIRTE Master robot.  
It uses **acados** for fast nonlinear optimisation and integrates with ROS 2 for deployment.

---

## Folder structure

- `generate_data/`
  - `generate_training_data.py`: generate training data.
  - `generate_validation_data.py`: generate validation data.
  - `log_data.py`: log measurements to a `.csv` file.
  - `stop_wheels.py`: emergency stop script to set all wheel speeds to zero.
- `control/`
  - Tube-based NMPC ROS 2 package.
  - `run_tube_mpc.sh`: build, source, and run the ROS 2 package.
  - `src/generate_ocp.py`: defines the NMPC nominal controller.
  - `c_generated_code/` and `acados_ocp.json`: acados-generated solver artifacts.
  - `include/`: required header files.
  - `src/tube_nmpc.cpp`: tube-based NMPC implementation. Offline simulation code is commented out. It integrates the extended kinematic model with RK4 and applies a fictitious disturbance to mimic the real system.
  - `src/acados_interface.cpp`: solves the optimisation problem.
  - `src/main.cpp`: node entry point.
  - `log_results.py`: log results to a `.csv` file.
  - `stop_wheels.sh`: emergency stop script to set all wheel speeds to zero.
- `baseline_controller/`
  - Baseline NMPC controller implementation.
  - Originally developed on a separate Git branch.
  - Folder structure mirrors `control/`.
---

## Prerequisites on robot

### 1. Install acados
Clone and build `acados` in the workspace `src/` directory (outside this branch):

```bash
git clone --recurse-submodules https://github.com/acados/acados.git
cd acados
mkdir build && cd build
cmake ..
make
sudo make install
```

### 2. Install CasADi
```bash
pip install casadi
```

---

###  Solver generation

The acados OCP and solver are defined in Python in `generate_ocp.py`. When it is executed, C code is generated in ``c_generated_code/``.

If the Python source file is modified (ex. for tuning NMPC weights), the generated C solver must be rebuilt:
```bash
cd c_generated_code
make
```

###  Build the ROS 2 package

Either use the script:
```bash
source run_tube_mpc.sh
```
Or to build, source and run, enter the individual commands:

```bash
colcon build --packages-select tube_mpc --cmake-clean-cache
source install/setup.bash
```
Then run the controller with:
```bash
ros2 run tube_mpc tube_mpc_node
```

---
## Note

The `SetSpeedMultiple` service was used during data generation to command wheel speeds simultaneously.

For closed-loop control, this proved unreliable, so the node publishes to individual wheel topics instead.