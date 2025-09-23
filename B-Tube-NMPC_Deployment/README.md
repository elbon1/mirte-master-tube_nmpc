# Tube-NMPC deployment on MIRTE Master

This package provides the real-time tube-based NMPC controller for the MIRTE Master robot.  
It uses **acados** for fast nonlinear optimisation and integrates with ROS 2 for deployment.

---

## Folder structure

- `control/`
  - Tube-based NMPC implementation.
  - Offline simulation code is present but commented. It integrates the extended kinematic model with RK4 and applies a fictitious disturbance to mimic the real system.
- `basline_controller/`
  - Baseline NMPC controller implementation.
  - Originally developed on a separate Git branch.
- `generate_data/`
  - Python scripts to obtain training data.
  - Logs measurements to CSV files.

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

