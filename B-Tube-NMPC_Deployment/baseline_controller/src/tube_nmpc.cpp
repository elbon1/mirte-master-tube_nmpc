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

void TubeNMPC::initParameters() {

  LB_vec = LB * VectorXd::Ones(nu);
  UB_vec = UB * VectorXd::Ones(nu);

  // Initialisation
  xref_loc = MatrixXd::Zero(Np + 1, MIRTE_NX);
  last_u_real = VectorXd::Zero(nu);
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

    const double omega = * (v_tan / R); // rad/s
    const double T     = 2.0 * M_PI * R / v_tan; // seconds for full circle

    cx = x0 - R * std::sin(yaw0);
    cy = y0 + R * std::cos(yaw0);

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

void TubeNMPC::setCurrentPose(const Eigen::Vector3d& pose) {
  current_pose_ = pose;
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
    std::cout << "xrefloc: " << xref_loc.row(0) << std::endl;
    std::cout << "current x: " << current_pose_.transpose() << std::endl;

    // Set state and reference for the NMPC solver
    acados_interface_->setState(current_pose_, LB_vec, UB_vec);
    acados_interface_->setReference(xref_loc, LB_vec, UB_vec);

    // Solve for nominal control input
    VectorXd u_real = acados_interface_->solve();
    std::cout << " u_real: "<< u_real.transpose() << std::endl;
    last_u_real = u_real;

    // Publish reference trajectory
    publish_reference_pose(xref_loc.row(0));
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

void TubeNMPC::sendWheelSpeeds(int fl, int fr, int rr, int rl) {
  static thread_local std_msgs::msg::Int32 msg;

  msg.data = fl; front_left_pub_->publish(msg);
  msg.data = fr; front_right_pub_->publish(msg);
  msg.data = rr; rear_right_pub_->publish(msg);
  msg.data = rl; rear_left_pub_->publish(msg);
}

void TubeNMPC::publish_reference_pose(const Eigen::VectorXd& xref_loc) {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->get_clock()->now();
    pose_msg.header.frame_id = "map";

    pose_msg.pose.position.x = xref_loc(0);
    pose_msg.pose.position.y = xref_loc(1);
    pose_msg.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, xref_loc(2));  // Yaw only
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    reference_publisher_->publish(pose_msg);
}
