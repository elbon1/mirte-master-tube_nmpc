#include "tube_nmpc.h"
#include "acados_interface.h"

TubeNMPC::TubeNMPC() : Node("tube_mpc"),
  Ts(0.1), Np(10), Nu(Np), nx(3), nu(4), LB(-100), UB(100)
{
  // Publishers to motor speed topics
  front_left_pub_ = this->create_publisher<std_msgs::msg::Int32>("/io/motor/front_left/speed", 10);
  front_right_pub_ = this->create_publisher<std_msgs::msg::Int32>("/io/motor/front_right/speed", 10);
  rear_left_pub_ = this->create_publisher<std_msgs::msg::Int32>("/io/motor/rear_left/speed", 10);
  rear_right_pub_ = this->create_publisher<std_msgs::msg::Int32>("/io/motor/rear_right/speed", 10);

  // Publisher for reference and nominal trajectories for logging
  reference_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("reference_trajectory", 10);
  nominal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("nominal_trajectory", 10);

  RCLCPP_INFO(this->get_logger(), "Waiting for all motor subscribers...");

  while (rclcpp::ok() &&
        (front_left_pub_->get_subscription_count() == 0 ||
          front_right_pub_->get_subscription_count() == 0 ||
          rear_left_pub_->get_subscription_count() == 0 ||
          rear_right_pub_->get_subscription_count() == 0)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  RCLCPP_INFO(this->get_logger(), "All motor subscribers found. Publishing...");

  // Initialise acados solver instance
  acados_interface_ = std::make_unique<AcadosInterface>();
  if (!acados_interface_->initialise()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialise Acados interface");
        rclcpp::shutdown();
    }
  
}

void TubeNMPC::initParameters()
{
  // Initial tightened bounds
  LB_tightened = 0.9 * LB * VectorXd::Ones(nu);
  UB_tightened = -LB_tightened;

  // DLQR Weights for ancillary control
  Q_lqr = Vector3d(1000, 1000, 1000).asDiagonal();
  R_lqr = 0.001 * MatrixXd::Identity(nu, nu);

  // Maximum disturbance obtained from data
  w_max = VectorXd::Zero(nx);
  w_max << 0.04, 0.03, 0.09;

  // Histories initialisation
  x_nomHistory = MatrixXd::Zero(Nt+1, nx);
  xref_loc = MatrixXd::Zero(Np + 1, MIRTE_NX);
  last_u_real = VectorXd::Zero(nu);
  
  // DLQR
  max_iter = 10000; 
  tol = 1e-9;  

  // RPI
  A_cl = vector<MatrixXd>(Nt, MatrixXd::Zero(nx, nx));
  P = vector<MatrixXd>(Nt, MatrixXd::Zero(nx, nx));
  err = MatrixXd::Zero(Nt, nx);
  rpi = vector<VectorXd>(Nt+1, VectorXd::Zero(nx)); 

  // FOR SIMULATION ONLY:
  x_realsim = MatrixXd::Zero(Nt+1, nx);
}

MatrixXd TubeNMPC::generateReferenceTrajectoryFromVelocity(const RowVectorXd& x0, 
                  double vel_x, double vel_y, double yaw_rate, double t_ref)
{
  int t = static_cast<int>(t_ref / Ts + 1);

  MatrixXd xref = MatrixXd::Zero(t, nx);
  double x = x0(0);
  double y = x0(1);
  double yaw = x0(2);

  int ramp_steps = t / 20;   // ramp up velocity along trajectory

  for (int i = 0; i < t; ++i) {
      xref(i, 0) = x;
      xref(i, 1) = y;
      xref(i, 2) = yaw;

      // Ramp factor from 0 to 1
      double alpha = (i < ramp_steps) ? (static_cast<double>(i) / ramp_steps) : 1.0;

      // Apply ramp to velocities
      double vx_cmd = alpha * vel_x;
      double vy_cmd = alpha * vel_y;
      double yaw_cmd = alpha * yaw_rate;

      // Transform local velocity to world frame
      double dx = (vx_cmd * std::cos(yaw) - vy_cmd * std::sin(yaw)) * Ts;
      double dy = (vx_cmd * std::sin(yaw) + vy_cmd * std::cos(yaw)) * Ts;
      double dyaw = yaw_cmd * Ts;

      x += dx;
      y += dy;
      yaw += dyaw;
  }

  return xref;
}

MatrixXd generateCircleNoHeadingChangeFull(const Vector3d& current_pose, double R, 
        double v_tan, double dt)
{
    MatrixXd out; 
    // Empty if incorrect inputs
    if (R <= 0.0 || v_tan <= 0.0 || dt <= 0.0) return out;

    const double x0   = current_pose(0);
    const double y0   = current_pose(1);
    const double yaw0 = current_pose(2);

    const double omega = v_tan / R; // rad/s
    const double T     = 2.0 * M_PI * R / v_tan; // seconds for full circle

    const double cx = x0 - R * std::sin(yaw0);
    const double cy = y0 + R * std::cos(yaw0);

    // Angle on the circle at t=0 so we start at current (x0,y0)
    const double theta0 = std::atan2(y0 - cy, x0 - cx);

    // Determine N to obtain a full circle based on the specifications
    const int N = static_cast<int>(std::ceil(T / dt));
    out.resize(N + 1, 6);

    for (int k = 0; k <= N; ++k) {
        const double t  = k * dt;
        const double th = theta0 + omega * t;

        const double cth = std::cos(th);
        const double sth = std::sin(th);

        const double x  = cx + R * cth;
        const double y  = cy + R * sth;

        // World-frame tangent velocity - heading fixed so wyaw = 0
        const double vx = -R * omega * sth;
        const double vy =  R * omega * cth;

        out(k,0) = x;
        out(k,1) = y;
        out(k,2) = yaw0; // fixed heading
        out(k,3) = vx; // world-frame vx
        out(k,4) = vy; // world-frame vy
        out(k,5) = 0.0; // wyaw
    }
    return out;
}

RowVectorXd TubeNMPC::generateReferenceTrajectory()
{
  // Uncomment for the chosen trajectory 

  // // Straight line
  // xref = generateReferenceTrajectoryFromVelocity(current_pose_, 0.3, 0.0, 0.0, 10.0);

  // // Rectangular path
  // MatrixXd xref1 = generateReferenceTrajectoryFromVelocity(current_pose_, 0.3, 0.0, 0.0, 5.0);
  // MatrixXd xref1_pause = generateReferenceTrajectoryFromVelocity(xref1.row(xref1.rows() - 1), 0.0, 0.0, 0.0, 0.3);
  // MatrixXd xref2 = generateReferenceTrajectoryFromVelocity(xref1_pause.row(xref1_pause.rows() - 1), 0.0, 0.2, 0.0, 5.0);
  // MatrixXd xref2_pause = generateReferenceTrajectoryFromVelocity(xref2.row(xref2.rows() - 1), 0.0, 0.0, 0.0, 0.3);
  // MatrixXd xref3 = generateReferenceTrajectoryFromVelocity(xref2_pause.row(xref2_pause.rows() - 1), -0.3, 0.0, 0.0, 5.0);
  // MatrixXd xref3_pause = generateReferenceTrajectoryFromVelocity(xref3.row(xref3.rows() - 1), 0.0, 0.0, 0.0, 0.3);
  // MatrixXd xref4 = generateReferenceTrajectoryFromVelocity(xref3_pause.row(xref3_pause.rows() - 1), 0.0, -0.2, 0.0, 5.0);
  // xref = vcat({xref1, xref1_pause, xref2, xref2_pause, xref3, xref3_pause, xref4});

  // Circular path fixed heading
  double R = 1.0;      // meters
  double v_tan = 0.2;  // m/s along the circle
  MatrixXd xref_all = generateCircleNoHeadingChangeFull(current_pose_, R, v_tan, Ts);
  xref = xref_all.leftCols<3>(); 

  Nt = xref.rows(); // Get number of samples of trajectory
  cout << "Number of samples in reference trajectory: " << Nt << endl;
  initParameters(); // Initialise parameters

  if (xref.rows() == 0)
    return RowVectorXd::Zero(nx);  // In case something goes wrong
  return xref.row(xref.rows() - 1);
}

// Vertically concatenate matrices with the same number of columns
inline Eigen::MatrixXd vcat(const std::vector<Eigen::MatrixXd>& blocks)
{
  if (blocks.empty()) return Eigen::MatrixXd();

  const Eigen::Index nx = blocks.front().cols();
  Eigen::Index total_rows = 0;
  for (const auto& M : blocks) {
      if (M.cols() != nx) throw std::runtime_error("column mismatch in vcat");
      total_rows += M.rows();
  }

  Eigen::MatrixXd out(total_rows, nx);
  Eigen::Index r = 0;
  for (const auto& M : blocks) {
      out.block(r, 0, M.rows(), nx) = M;
      r += M.rows();
  }
  return out;
}
// Make a window of Np+1 rows of the reference trajectory for NMPC
inline Eigen::MatrixXd makeWindow(const Eigen::MatrixXd& xref, int k, int Np) 
{
  const Eigen::Index nx = xref.cols();
  Eigen::MatrixXd xref_loc(Np + 1, nx);
  for (int i = 0; i <= Np; ++i) {
    int idx = std::min<int>(k + i, static_cast<int>(xref.rows()) - 1);
    xref_loc.row(i) = xref.row(idx);
  }
  return xref_loc;
}

// Solve DARE solution P and DLQR gain K 
void TubeNMPC::solveDARE(const MatrixXd& A, const MatrixXd& B, const MatrixXd& Q, const MatrixXd& R, MatrixXd& P, MatrixXd& K, int max_iter, double tol)
{
    P = Q;
    MatrixXd P_prev;

    for (int i = 0; i < max_iter; i++)
    {
        P_prev = P;
        MatrixXd BT_P = B.transpose() * P;
        MatrixXd K = (R + BT_P * B).inverse() * (BT_P * A);
        P = A.transpose() * P * A - (A.transpose() * P * B) * ((B.transpose() * P * B + R).inverse()) * (B.transpose() * P * A )  + Q;

        if ((P - P_prev).norm() < tol){
          //std::cout << "Converged P" << std::endl;
          break;
        }

    }

    K = (R + B.transpose() * P * B).inverse() * (B.transpose() * P * A);
}

void TubeNMPC::setCurrentPose(const Eigen::Vector3d& pose) {
  current_pose_ = pose;
  // // FOR SIMULATION ONLY:
  // current_pose = x_realsim.row(0); 
}

void TubeNMPC::setInitialPose(const Eigen::Vector3d& pose) {
  x_nomHistory.row(0) = pose;
  std::cout << "Initial Pose: " << x_nomHistory.row(current_step_) << std::endl;
}

VectorXd TubeNMPC::getLastControlInput() const {
  return last_u_real;
}

void TubeNMPC::setCurrentStep(int step) {
  current_step_ = step;
}

bool TubeNMPC::runControlLoop(int step) {
  current_step_ = step;
  try {

    xref_loc = makeWindow(xref, current_step_, Np);
    std::cout << "Current x_ref: " << xref_loc.row(0) << std::endl;
    std::cout << "current x: " << current_pose_.transpose() << std::endl;
    std::cout << "RPI:" << rpi[current_step_].transpose() << std::endl;

    // Set state and reference for the NMPC solver
    acados_interface_->setState(current_pose_, rpi[current_step_], LB_tightened, UB_tightened);
    acados_interface_->setReference(xref_loc, LB_tightened, UB_tightened);

    // Solve for nominal control input
    VectorXd u_nom = acados_interface_->solve();
    VectorXd x0_nom = acados_interface_->getx0();
    x_nomHistory.row(current_step_) = x0_nom.transpose();

    // std::cout << "x0nom: " << x0_nom.transpose() << std::endl;
    x_nomHistory.row(current_step_+1) = (propagateDiscreteDynamics(x0_nom, u_nom, VectorXd::Zero(nx))).transpose(); // For plotting

    // Linearise around the current state with last known control
    MatrixXd A_ct(nx, nx), B_ct(nx, nu);
    lineariseSindy(x0_nom, u_nom, A_ct, B_ct);

    // Discretise using 1st order Forward-Euler
    MatrixXd I = MatrixXd::Identity(nx, nx);
    MatrixXd A_dt = I + Ts * A_ct;
    MatrixXd B_dt = Ts * B_ct;

    // Solve DARE for DLQR
    MatrixXd P_lqr, K;
    solveDARE(A_dt, B_dt, Q_lqr, R_lqr, P_lqr, K, max_iter, tol);
    // std::cout << "P_lqr :" << P_lqr << std::endl;

    // // FOR SIMULATION ONLY:
    // disturbance(j) = (2.0 * ((double) rand() / RAND_MAX) - 1.0) * w_max(j);
    // x_realsim.row(current_step_) = (propagateDiscreteDynamics(current_pose_, u_real, disturbance)).transpose();
    // current_pose_ = x_realsim.row(current_step_);

    // FOR REAL IMPLEMENTATION:
    // Compute ancillary control law
    VectorXd err = current_pose_ - x0_nom;
    VectorXd u_real = u_nom - K * err;
    // std::cout << "unom: " << u_nom.transpose() << std::endl;
    // std::cout << "err: " << err.transpose() << std::endl;

    std::cout << " u_real: "<< u_real.transpose() << std::endl;
    last_u_real = u_real;

    // Closed-loop DLQR system
    A_cl[current_step_] = A_dt - B_dt * K;

    // Just a confirmation that the closed-loop is stable
    if (isStable(A_cl[current_step_]) == 0) {
      std::cout << "DLQR controller is not stabilising." << std::endl;
  }
    // Approximate RPI set
    MatrixXd Phi_abs = A_cl[current_step_].cwiseAbs();
    rpi[current_step_+1] = Phi_abs * rpi[current_step_] + w_max * Ts;
    // std::cout << "rpi :" << rpi[current_step_].transpose() << std::endl;

    // Compute new tightened sets
    VectorXd du_bound = K.cwiseAbs() * rpi[current_step_]; 
    // std::cout << "du_bound: " << du_bound.transpose() << std::endl; 
    LB_tightened = VectorXd::Constant(nu, LB) + du_bound;
    UB_tightened = VectorXd::Constant(nu, UB) - du_bound;
    // std::cout << "LB_tightened: " << LB_tightened.transpose() << std::endl;
    // std::cout << "UB_tightened: " << UB_tightened.transpose() << std::endl;

    // Publish reference and nominal trajectories
    publish_reference_pose(xref_loc.row(0));
    publish_nominal_pose(x0_nom);
    return true;

  } catch (const std::exception& e)
  {
    // If there is an error, send zero PWM commands
    RCLCPP_ERROR(this->get_logger(), "runControlLoop() failed: %s", e.what());

    // Send zero velocity commands
    this->sendWheelSpeeds(0, 0, 0, 0);
    RCLCPP_INFO(this->get_logger(), "Sent zero wheel speeds due to ACADOS failure.");

    // Add short delay to ensure message is published
    rclcpp::sleep_for(std::chrono::milliseconds(100));

    // Exit the node
    rclcpp::shutdown();
    std::exit(EXIT_FAILURE);
}
}

// G(x) maps body-frame velocities into world rates - needed for simulating disturbance
Eigen::Matrix3d G_of(const VectorXd& x) {
  const double theta = x(2); // yaw = x3
  Matrix3d G;
  G <<  std::cos(theta), -std::sin(theta), 0.0,
        std::sin(theta),  std::cos(theta), 0.0,
        0.0,            0.0,           1.0;
  return G;
}

VectorXd TubeNMPC::propagateDiscreteDynamics(const VectorXd &x, const VectorXd &u, const VectorXd &disturbance) {
  
  const Matrix3d G1 = G_of(x);
  const VectorXd f1 = sindyc(x, u) + G1 * disturbance;

  const VectorXd x2 = x + 0.5 * Ts * f1;
  const Matrix3d G2 = G_of(x2);
  const VectorXd f2 = sindyc(x2, u) + G2 * disturbance;

  const VectorXd x3 = x + 0.5 * Ts * f2;
  const Matrix3d G3 = G_of(x3);
  const VectorXd f3 = sindyc(x3, u) + G3 * disturbance;

  const VectorXd x4 = x + Ts * f3;
  const Matrix3d G4 = G_of(x4);
  const VectorXd f4 = sindyc(x4, u) + G4 * disturbance;
  return x + (Ts / 6.0) * (f1 + 2 * f2 + 2 * f3 + f4);
}

VectorXd TubeNMPC::sindyc(const VectorXd &x, const VectorXd &u) {
  // Extended kinematic model
  double x3 = x(2);
  double u1 = u(0), u2 = u(1), u3 = u(2), u4 = u(3);

  double x1dot = -0.00016598*u1 -0.00044484*u2 + 0.00028542*u3 + 0.00021938*u4 
                  + 0.00052793*sin(x3) * u1 + 0.0022203*cos(x3) * u1 -0.00066344*sin(x3) * u2 + 0.0023484*cos(x3) * u2
                  + 0.00087885*sin(x3) * u3 + 0.0013426*cos(x3) * u3 -0.00075726*sin(x3) * u4 + 0.00081618*cos(x3) * u4;

  double x2dot = 0.0004803*u1 + 0.000149*u2 -0.00029443*u3 -0.00045783*u4 
                  + 0.0013242*sin(x3) * u1 -0.00085305*cos(x3) * u1 + 0.0016074*sin(x3) * u2 + 0.00068381*cos(x3) * u2
                  + 0.0023134*sin(x3) * u3 -0.00074272*cos(x3) * u3 + 0.0021253*sin(x3) * u4 + 0.00088697*cos(x3) * u4;

  double x3dot = -0.0043797*u1 + 0.008181*u2 + 0.0046648*u3 -0.0078721*u4 
                  -7.1607e-05*u1*u1 -7.2158e-05*u1*u2 + 0.00011008*u1*u3 + 1.6506e-05*u1*u4 
                  + 5.1765e-05*u2*u3 -2.1046e-05*u2*u4 -3.0088e-05*u3*u3 + 2.7602e-05*u4*u4;

  return Vector3d(x1dot, x2dot, x3dot);
}

void TubeNMPC::lineariseSindy(const VectorXd &x, const VectorXd &u, MatrixXd &A, MatrixXd &B)
{
  // Local linearisation - Jacobian matrices A & B
  A = MatrixXd::Zero(3, 3);
  B = MatrixXd::Zero(3, 4);

  double x3 = x(2);
  double s = sin(x3);
  double c = cos(x3);

  double u1 = u(0), u2 = u(1), u3 = u(2), u4 = u(3);

  A(0,2) = (0.00052793*c - 0.0022203*s)*u1
          + (-0.00066344*c - 0.0023484*s)*u2
          + (0.00087885*c - 0.0013426*s)*u3
          + (-0.00075726*c - 0.00081618*s)*u4;
  A(1,2) = (0.0013242*c + 0.00085305*s)*u1
        + (0.0016074*c - 0.00068381*s)*u2
        + (0.0023134*c + 0.00074272*s)*u3
        + (0.0021253*c - 0.00088697*s)*u4;

  B(0,0) = -0.00016598 + 0.00052793*s + 0.0022203*c;
  B(0,1) = -0.00044484 - 0.00066344*s + 0.0023484*c;
  B(0,2) =  0.00028542 + 0.00087885*s + 0.0013426*c;
  B(0,3) =  0.00021938 - 0.00075726*s + 0.00081618*c;

  B(1,0) =  0.0004803 + 0.0013242*s - 0.00085305*c;
  B(1,1) =  0.000149  + 0.0016074*s + 0.00068381*c;
  B(1,2) = -0.00029443 + 0.0023134*s - 0.00074272*c;
  B(1,3) = -0.00045783 + 0.0021253*s + 0.00088697*c;

  B(2,0) = -0.0043797 - 2.0*7.1607e-05*u1 - 7.2158e-05*u2 + 1.1008e-04*u3 + 1.6506e-05*u4;
  B(2,1) =  0.008181  - 7.2158e-05*u1 + 5.1765e-05*u3 - 2.1046e-05*u4;
  B(2,2) =  0.0046648 + 1.1008e-04*u1 + 5.1765e-05*u2 - 2.0*3.0088e-05*u3;
  B(2,3) = -0.0078721 + 1.6506e-05*u1 - 2.1046e-05*u2 + 2.0*2.7602e-05*u4;
}

void TubeNMPC::sendWheelSpeeds(int fl, int fr, int rr, int rl) 
{
  static thread_local std_msgs::msg::Int32 msg;

  msg.data = fl; front_left_pub_->publish(msg);
  msg.data = fr; front_right_pub_->publish(msg);
  msg.data = rr; rear_right_pub_->publish(msg);
  msg.data = rl; rear_left_pub_->publish(msg);
}

void TubeNMPC::publish_reference_pose(const Eigen::VectorXd& xref_loc)
{
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = this->get_clock()->now();
  pose_msg.header.frame_id = "map";

  pose_msg.pose.position.x = xref_loc(0);
  pose_msg.pose.position.y = xref_loc(1);
  pose_msg.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, xref_loc(2)); // Yaw only
  pose_msg.pose.orientation.x = q.x();
  pose_msg.pose.orientation.y = q.y();
  pose_msg.pose.orientation.z = q.z();
  pose_msg.pose.orientation.w = q.w();

  reference_publisher_->publish(pose_msg);
}

void TubeNMPC::publish_nominal_pose(const Eigen::VectorXd& x_nom_0)
{
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = this->get_clock()->now();
  pose_msg.header.frame_id = "map";

  pose_msg.pose.position.x = x_nom_0(0);
  pose_msg.pose.position.y = x_nom_0(1);
  pose_msg.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, x_nom_0(2)); // Yaw only
  pose_msg.pose.orientation.x = q.x();
  pose_msg.pose.orientation.y = q.y();
  pose_msg.pose.orientation.z = q.z();
  pose_msg.pose.orientation.w = q.w();

  nominal_publisher_->publish(pose_msg);
}

bool isStable(const MatrixXd& Acl) 
{
  Eigen::EigenSolver<MatrixXd> solver(Acl);
  VectorXcd eigvals = solver.eigenvalues();

  for (int i = 0; i < eigvals.size(); ++i) {
      if (std::abs(eigvals[i]) >= 1.0) {
          return false; // Unstable
      }
  }
  return true; // All eigenvalues inside unit circle
}
