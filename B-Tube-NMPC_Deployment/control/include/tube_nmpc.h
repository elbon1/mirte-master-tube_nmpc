#ifndef TUBE_NMPC_H
#define TUBE_NMPC_H

#include <rclcpp/rclcpp.hpp>
#include "acados_interface.h"
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <fstream>
#include <unsupported/Eigen/KroneckerProduct>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp> // To publish Reference trajectory for logging
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "std_msgs/msg/int32.hpp"
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <iostream>


using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Matrix3d;
using std::vector;
using Eigen::RowVectorXd;
using Eigen::Map;
using std::floor;

bool isStable(const MatrixXd& Acl);

MatrixXd vcat(const std::vector<Eigen::MatrixXd>& blocks);
MatrixXd makeWindow(const Eigen::MatrixXd& xref, int k, int Np);
Matrix3d G_of(const VectorXd& x);

MatrixXd generateCircleNoHeadingChangeFull(const Vector3d& current_pose,
            double R, double v_tan, double dt);

class TubeNMPC : public rclcpp::Node {
public:
  TubeNMPC();

  RowVectorXd generateReferenceTrreadmajectory();
  bool runControlLoop(int step);

  // State interface
  Vector3d current_pose_ = Vector3d::Zero();
  void setCurrentPose(const Vector3d& pose);
  void setInitialPose(const Vector3d& pose);
  void setCurrentStep(int step);

  VectorXd getLastControlInput() const;
  int getNt() const { return Nt; }

  // Publish wheel commands
  void sendWheelSpeeds(int fl, int fr, int rr, int rl);
  
  
private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr front_left_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr front_right_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr rear_left_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr rear_right_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr reference_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr nominal_publisher_;
  
  void publish_reference_pose(const Eigen::VectorXd& xref_loc);
  void publish_nominal_pose(const Eigen::VectorXd& x_nom_0);

  // Parameters
  double Ts, tol;
  int Np, Nu, nx, nu, Nt, max_iter;
  VectorXd w_max;

  // Matrices and constraints
  MatrixXd Q_lqr, R_lqr, err;
  double LB, UB;
  VectorXd LB_tightened, UB_tightened;
  vector <MatrixXd> A_cl, P;
  vector<VectorXd> rpi;

  // Histories
  MatrixXd xref, xref_loc, x_nomHistory, x_realsim;
  MatrixXd u_nom;
  VectorXd last_u_real;
  int current_step_ = 0;

  std::unique_ptr<AcadosInterface> acados_interface_;

  // Ancillary controller
  void lineariseSindy(const VectorXd &x, const VectorXd &u, MatrixXd &A, MatrixXd &B);
  void solveDARE(const MatrixXd& A, const MatrixXd& B, const MatrixXd& Q, const MatrixXd& R, 
    MatrixXd& P, MatrixXd& K, int max_iter, double tol);

  // Functions
  void initParameters();
  MatrixXd generateReferenceTrajectoryFromVelocity(const RowVectorXd& x0, double vel_x, 
                                                    double vel_y, double yaw_rate, double t_ref);
  VectorXd propagateDiscreteDynamics(const VectorXd &x, const VectorXd &u, const VectorXd &disturbance);
  VectorXd sindyc(const VectorXd &x, const VectorXd &u);
};

#endif