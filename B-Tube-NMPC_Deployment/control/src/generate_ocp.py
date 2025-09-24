from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
import numpy as np
import casadi as ca

def create_model():
    model = AcadosModel()
    model.name = 'mirte'

    # Define symbolic variables
    x = ca.MX.sym('x', 3) # states: x, y, theta
    u = ca.MX.sym('u', 4) # control: u_1, u_2, u_3, u_4
    ud = ca.MX.sym('ud', 4) # input rate: ud_1, ud_2, ud_3, ud_4
    p = ca.MX.sym('p', 3) # parameters: 3 for x_ref trajectory

    # Extract variables
    x3 = x[2]
    u1, u2, u3, u4 = u[0], u[1], u[2], u[3]

    # Define system dynamics - Extended kinematic model from SINDYc
    x1dot = -0.00016598*u1 -0.00044484*u2 + 0.00028542*u3 + 0.00021938*u4 \
            + 0.00052793*ca.sin(x3) * u1 + 0.0022203*ca.cos(x3) * u1 \
            -0.00066344*ca.sin(x3) * u2 + 0.0023484*ca.cos(x3) * u2 \
            + 0.00087885*ca.sin(x3) * u3 + 0.0013426*ca.cos(x3) * u3 \
            -0.00075726*ca.sin(x3) * u4 + 0.00081618*ca.cos(x3) * u4
    
    x2dot = 0.0004803*u1 + 0.000149*u2 -0.00029443*u3 -0.00045783*u4 \
            + 0.0013242*ca.sin(x3) * u1 -0.00085305*ca.cos(x3) * u1 \
            + 0.0016074*ca.sin(x3) * u2 + 0.00068381*ca.cos(x3) * u2 \
            + 0.0023134*ca.sin(x3) * u3 -0.00074272*ca.cos(x3) * u3 \
            + 0.0021253*ca.sin(x3) * u4 + 0.00088697*ca.cos(x3) * u4
    
    x3dot = -0.0043797*u1 + 0.008181*u2 + 0.0046648*u3 -0.0078721*u4 \
            -7.1607e-05*u1*u1 -7.2158e-05*u1*u2 + 0.00011008*u1*u3 \
            + 1.6506e-05*u1*u4 + 5.1765e-05*u2*u3 -2.1046e-05*u2*u4 \
            -3.0088e-05*u3*u3 + 2.7602e-05*u4*u4

    f_expl = ca.vertcat(x1dot, x2dot, x3dot, ud)

    
    model.x = ca.vertcat(x, u)
    model.u = ud
    model.p = p
    model.f_expl_expr = f_expl

    # Cost expression: deviation from reference trajectory
    model.cost_y_expr = ca.vertcat(x - p, u, ud)
    model.cost_y_expr_e = x - p

    model.x_labels = ['x [m]', 'y [m]', 'theta [rad]']
    model.u_labels = ['u_1', 'u_2', 'u_3', 'u_4']
    model.t_label = '$t$ [s]'

    return model

def main():
    
    # Create OCP object to formulate the OCP
    ocp = AcadosOcp()
    model = create_model()
    ocp.model = model

    Ts = 0.1 # Sampling time
    nx = 3 # Number of states
    nu = 4 # Number of inputs
    ocp.dims.np = model.p.rows() # Must match the cost dimension (nx+nu)
    ocp.parameter_values = np.zeros(ocp.dims.np) # Initialise parameters

    N = 10  # NMPC prediction horizon
    Tf = 10.0  # Total simulation time (s) 
    additional_steps = 0 # To test convergence
    sim_steps = int(Tf / Ts) + 1 + additional_steps

    x0 = np.array([3.33445, 0.0, 0.0]) # Initial condition

    ## Reference trajectories

    # # Rectangle trajectory constant heading
    # step_a = 100
    # step_b = 40
    # x_ref_part1 = np.linspace(x0, x0+[3.0, 0.0, 0.0], step_a)
    # x_ref_part2 = np.linspace(x0+[3.0, 0.0, 0.0], x0+[3.0, 0.8, 0.0], step_b)
    # x_ref_part3 = np.linspace(x0+[3.0, 0.8, 0.0], x0+ [0.0, 0.8, 0.0], step_a)
    # x_ref_part4 = np.linspace(x0+[0.0, 0.8, 0.0], x0, sim_steps - 2 * step_a - step_b - additional_steps)
    # x_ref_part5 = np.tile(x0, (additional_steps, 1))
    # x_ref_traj = np.vstack((x_ref_part1, x_ref_part2, x_ref_part3, x_ref_part4, x_ref_part5))

    # Rectangle trajectory varying heading
    # step_a = 300
    # step_b = 300
    # x_ref_part1 = np.linspace([0.0, 0.0, 0.0], [4.0, 0.0, 0.0], step_a)
    # x_ref_part2 = np.linspace([4.0, 0.0, 0.0], [4.0, 1.0, np.pi/2], step_b)
    # x_ref_part3 = np.linspace([4.0, 1.0, np.pi/2], [0.0, 1.0, np.pi], step_a)
    # x_ref_part4 = np.linspace([0.0, 1.0, np.pi], [0.0, 0.0, 3*np.pi/2], sim_steps - 2 * step_a - step_b - additional_steps)
    # x_ref_part5 = np.tile([0.0, 0.0, 0.0], (additional_steps, 1))
    # x_ref_traj = np.vstack((x_ref_part1, x_ref_part2, x_ref_part3, x_ref_part4, x_ref_part5))

    # # Straight line
    x_ref_part1 = np.linspace(x0, x0 + [0.0, 2.0, 0.0], sim_steps - additional_steps)
    x_ref_part2 = np.tile(x0 + [0.0, 2.0, 0.0], (additional_steps, 1))
    x_ref_traj = np.vstack((x_ref_part1, x_ref_part2))

    # Set prediction horizon
    ocp.dims.N = N
    ocp.solver_options.N_horizon = N
    ocp.solver_options.tf = N * Ts
    
    # NMPC Weights
    Q_mat = 1e3*np.diag([1, 7, 10]) # Stage weighing matrix
    # Qf_mat = 1e4*np.diag([1, 1, 1]) # Initial terminal weighing matix
    Qf_mat = np.array([
    [5261.05, -1695.38, -119.571],
    [-1695.38,  5024.49,  32.4935],
    [-119.571,  32.4935,  1414.31]]) # Final termnial weighing matrix
    R_mat = 1e-2*np.diag([1, 1, 1, 1]) # Input cost weighing matrix
    Rd_mat = 1e-1*np.diag([1, 1, 1, 1]) # Input rate cost weighing matrix

    # Generate cost function
    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.cost.yref = np.zeros((nx+nu+nu,)) # Reference integrated within cost - set to 0
    ocp.cost.W = ca.diagcat(Q_mat, R_mat,Rd_mat).full() # Weighing matrices
    
    # Generate terminal cost function
    ocp.cost.cost_type_e = 'NONLINEAR_LS'
    ocp.cost.yref_e = np.zeros((nx,)) # Reference - set to 0
    ocp.cost.W_e = Qf_mat # Terminal cost matrix
    
    # Set constraints for input rate
    ud_max = 30.0   # Maximum input rate
    ocp.constraints.constr_type = 'BGH'
    ocp.constraints.idxbu = np.array([0,1,2,3])
    ocp.constraints.lbu  = -ud_max * np.ones(4)
    ocp.constraints.ubu  =  ud_max * np.ones(4)
    
    # Set constraints for "x" and control input 
    Fmax = 100 # Maximum input
    ocp.constraints.idxbx = np.array([0,1,2,3,4,5,6])
    ocp.constraints.lbx = np.array([-10e10,-10e10,-10e10,-Fmax, -Fmax, -Fmax, -Fmax])
    ocp.constraints.ubx = np.array([10e10, 10e10, 10e10, Fmax, Fmax, Fmax, Fmax])

    ocp.constraints.idxbx_0 = np.array([0, 1, 2, 3, 4, 5, 6])  # indices of states constrained at t=0
    ocp.constraints.lbx_0 = np.hstack([x0, -Fmax, -Fmax, -Fmax, -Fmax])
    ocp.constraints.ubx_0 = np.hstack([x0, Fmax, Fmax, Fmax, Fmax])

    # Terminal constraints
    ocp.constraints.constr_type_e = 'BGH'
    ocp.constraints.idxbx_e = np.array([0, 1, 2,3,4,5,6])
    ocp.constraints.lbx_e = np.array([-10e10, -10e10, -10e10, -Fmax, -Fmax, -Fmax, -Fmax])
    ocp.constraints.ubx_e = np.array([10e10,10e10,10e10, Fmax, Fmax, Fmax, Fmax])
    
    # Set options
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.sim_method_num_stages = 4 # RK4
    ocp.solver_options.sim_method_num_steps  = 3
    ocp.solver_options.nlp_solver_type = 'SQP'
    ocp.solver_options.qp_solver = 'FULL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.levenberg_marquardt = 1e-4
    ocp.solver_options.globalization = 'MERIT_BACKTRACKING'
    ocp.solver_options.line_search_use_sufficient_descent=1
    ocp.solver_options.alpha_min = 1e-2 
    ocp.solver_options.nlp_solver_max_iter = 1000 # More iterations
    ocp.solver_options.nlp_solver_tol_eq = 1e-4
    ocp.solver_options.nlp_solver_tol_stat = 1e-4
    ocp.solver_options.qp_solver_tol_stat = 1e-6 # QP feasibility tolerance
    
    # Generate acados solver artificats
    ocp_solver = AcadosOcpSolver(ocp, 
                            json_file="acados_ocp.json",
                            build=True, 
                            generate=True)
    
    # Simulate response to tune weights
    simX = np.zeros((sim_steps+1, nx))
    simU = np.zeros((sim_steps, nu))

    x_current = x0
    simX[0, :] = x_current
    rpi = np.array([0.002, 0.002, 0.001])

    # Initial bounds
    LB_0 = np.empty((N+1,7))
    UB_0 = np.empty((N+1,7))
    LB_0[0,0:3] = x_current - rpi
    UB_0[0,0:3] = x_current + rpi
    LB_0[0,3:7] = np.array([-Fmax, -Fmax, -Fmax, -Fmax])
    UB_0[0,3:7] = np.array([Fmax, Fmax, Fmax, Fmax])

    # Bounds through the horizon N+1 stages
    LB = np.empty((N+1,7))
    UB = np.empty((N+1,7))
    LB_e = np.empty((7))
    UB_e = np.empty((7))
    LB_e[3:7] = np.array([-Fmax, -Fmax, -Fmax, -Fmax])
    UB_e[3:7] = np.array([Fmax, Fmax, Fmax, Fmax])

    for k in range(1, N+1):
        LB[k,:] = [-10e10,-10e10,-10e10,-Fmax, -Fmax, -Fmax, -Fmax]
        UB[k,:] = [10e10,10e10,10e10,Fmax, Fmax, Fmax, Fmax]

    # Start simulation
    for t in range(sim_steps):
        print(f"\n===== Step {t} ===== ")
        print(f"Current state: {x_current}")
        print(f"Reference Trajectory: {x_ref_traj[t]}")
        
        # Set initial state
        LB_0[0,0:3] = x_current - rpi
        UB_0[0,0:3] = x_current + rpi
        
        ocp_solver.set(0, "lbx", LB_0[0,:])
        ocp_solver.set(0, "ubx", UB_0[0,:])

        # Set bounds for N+1 stages
        for k in range(1, N+1):
            ocp_solver.set(k, "lbx", LB[k])
            ocp_solver.set(k, "ubx", UB[k]) 

        # Set the reference trajectory for N+1 stages
        for k in range(N):
            idx = min(t + k, sim_steps-1)
            ocp_solver.set(k, "p", x_ref_traj[idx])
            
        idx_terminal = min(t + N, sim_steps-1)
        ocp_solver.set(N, "p", x_ref_traj[idx_terminal])

        # Solve OCP
        status = ocp_solver.solve()

        if status != 0:
            raise Exception(f'acados returned status {status}.')

        # Get and apply control
        u0 = ocp_solver.get(0, "x")[3:7]
        print(f"Optimal control u[0]: {u0}")
        simU[t, :] = u0

        # Get next state
        x_next = ocp_solver.get(1, "x")
        simX[t+1, :] = x_next[0:3]
        print(f"Predicted next state: {x_next}")
        x_current = x_next[0:3]

if __name__ == '__main__':
    main()