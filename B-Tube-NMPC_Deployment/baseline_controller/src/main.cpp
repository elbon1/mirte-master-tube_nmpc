#include "tube_nmpc.h"

std::atomic<bool> running(true); // Global flag to handle Ctrl+C

void signalHandler(int signum) {
  std::cout << "\nInterrupt signal (" << signum << ") received. Shutting down...\n";
  running = false;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::signal(SIGINT, signalHandler); // Ctrl+c handler

  auto node = std::make_shared<TubeNMPC>();

  Vector3d pose = Vector3d::Zero();
  std::atomic<bool> pose_received = false;

  // Odometry subscription
  auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
    "/mirte_base_controller/odom", 10,
    [&pose, &pose_received](const nav_msgs::msg::Odometry::SharedPtr msg) {
      double x = msg->pose.pose.position.x;
      double y = msg->pose.pose.position.y;

      tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      pose << x, y, yaw;
      pose_received = true;
    });

  // Wait for initial pose
  RCLCPP_INFO(node->get_logger(), "Waiting for initial pose...");
  while (rclcpp::ok() && !pose_received && running) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  RCLCPP_INFO(node->get_logger(), "Initial pose received.");

  int current_step_ = 0;
  node->setCurrentPose(pose);

  // Generate a reference trajectory from current pose
  Eigen::RowVectorXd goal =  node->generateReferenceTrajectory();
  node->setInitialPose(pose);

  int Nt = node->getNt();

  // Main control loop at 10 Hz
  rclcpp::Rate rate(10);
  while (rclcpp::ok() && running) {
    bool fresh_odom = false;

    // Wait for new odometry
    while (!fresh_odom && rclcpp::ok() && running) {
      rclcpp::spin_some(node);
      if (pose_received.exchange(false)) {
        fresh_odom = true;
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    }

    // Run control using latest pose
    node->setCurrentPose(pose);

    if (current_step_ == Nt) {
      std::cout << "Duration reached. Stopping..." << std::endl;
      break;
    }

    node->runControlLoop(current_step_);

    auto u = node->getLastControlInput();

    // Apply minimum magnitude threshold
    for (int i = 0; i < u.size(); ++i) {
      if (std::abs(u(i)) < 10) {
        u(i) = 0;
      }
    }
 
    node->sendWheelSpeeds(u(0),u(1),u(2),u(3));

     // Increase step counter
    current_step_++;
    node->setCurrentStep(current_step_);
    rate.sleep();
  }

  // Send zero velocity to all motors on shutdown
  std::cout << "Sending zero speed to all motors..." << std::endl;
  node->sendWheelSpeeds(0, 0, 0, 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Give time for messages to send

  rclcpp::shutdown();
  return 0;
}
