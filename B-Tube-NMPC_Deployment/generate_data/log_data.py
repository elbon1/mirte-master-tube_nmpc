import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from datetime import datetime
import csv
import numpy as np


def get_yaw_from_quaternion(orientation):
    # Convert quaternion to yaw angle
    siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cosy_cosp = 1 - 2 * (orientation.y ** 2 + orientation.z ** 2)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return yaw

class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/mirte_base_controller/odom', self.odom_callback, 10)
        self.left_front_sub = self.create_subscription(Int32, '/wheel_pwm/left_front', self.left_front_callback, 10)
        self.right_front_sub = self.create_subscription(Int32, '/wheel_pwm/right_front', self.right_front_callback, 10)
        self.left_rear_sub = self.create_subscription(Int32, '/wheel_pwm/left_rear', self.left_rear_callback, 10)
        self.right_rear_sub = self.create_subscription(Int32, '/wheel_pwm/right_rear', self.right_rear_callback, 10)

        self.reference_sub = self.create_subscription(PoseStamped, '/reference_trajectory', self.reference_callback, 10)
        self.last_reference = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

        # Open CSV file for writing
        self.csv_file = open('data_log.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
        'Timestamp', 'X', 'Y', 'Yaw', 'VX', 'VY', 'VYaw',
        'Left_Front_Input', 'Right_Front_Input', 'Left_Rear_Input', 'Right_Rear_Input',
        'X_ref', 'Y_ref', 'Yaw_ref'
    ])

        # Initialise logged data
        self.yaw_history = [] # Store previous yaw values
        self.last_odom = None
        self.last_wheel_inputs = {'left_front': 0, 'right_front': 0, 'left_rear': 0, 'right_rear': 0}

        # Create a timer to log data every 0.1s
        self.timer = self.create_timer(0.1, self.timer_callback)

    # Store the latest PWM input values
    def left_front_callback(self, msg):
        self.last_wheel_inputs['left_front'] = msg.data

    def right_front_callback(self, msg):
        self.last_wheel_inputs['right_front'] = msg.data

    def left_rear_callback(self, msg):
        self.last_wheel_inputs['left_rear'] = msg.data

    def right_rear_callback(self, msg):
        self.last_wheel_inputs['right_rear'] = msg.data

    # Store the latest odometry data
    def odom_callback(self, msg):
        yaw = get_yaw_from_quaternion(msg.pose.pose.orientation)
        self.yaw_history.append(yaw)
        if len(self.yaw_history) > 1:
            self.yaw_history = list(np.unwrap(self.yaw_history)) # Apply unwrap
        
        yaw_unwrapped = self.yaw_history[-1]
        self.last_odom = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'yaw': yaw_unwrapped,
            'vx': msg.twist.twist.linear.x,
            'vy': msg.twist.twist.linear.y,
            'vyaw': msg.twist.twist.angular.z
        }

    # Store reference trajectory
    def reference_callback(self, msg):
        quat = msg.pose.orientation
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y ** 2 + quat.z ** 2)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        self.last_reference = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'yaw': yaw
    }

    def timer_callback(self):
        """Log data to the CSV file."""
        current_time = datetime.now()

        # Extract odometry data
        odom = self.last_odom if self.last_odom else {'x': 0, 'y': 0, 'yaw': 0, 'vx': 0, 'vy': 0, 'vyaw': 0}

        # Extract PWM input values
        left_front_pwm = self.last_wheel_inputs['left_front']
        right_front_pwm = self.last_wheel_inputs['right_front']
        left_rear_pwm = self.last_wheel_inputs['left_rear']
        right_rear_pwm = self.last_wheel_inputs['right_rear']

        # Log PWM for debugging
        self.get_logger().info(f"Writing PWM values to CSV: LF={left_front_pwm}, RF={right_front_pwm}, LR={left_rear_pwm}, RR={right_rear_pwm}")

        # Extract reference
        xref = self.last_reference['x']
        yref = self.last_reference['y']
        yawref = self.last_reference['yaw']

        # Write to CSV
        self.csv_writer.writerow([
            current_time, odom['x'], odom['y'], odom['yaw'], odom['vx'], odom['vy'], odom['vyaw'],
            left_front_pwm, right_front_pwm, left_rear_pwm, right_rear_pwm,
            xref, yref, yawref
        ])
        
        # Ensure data is saved immediately
        self.csv_file.flush()

    def destroy_node(self):
        self.csv_file.close() # Close CSV file
        super().destroy_node()

def main():
    rclpy.init()
    data_logger = DataLogger() # Create DataLogger instance

    try:
        rclpy.spin(data_logger) # Keep the node running
    except KeyboardInterrupt:
        # Handle Ctrl+C
        data_logger.get_logger().info('Ctrl+C detected. Stopping wheels.')
    finally:
        data_logger.destroy_node()
        rclpy.shutdown() # Shutdown ROS 2

if __name__ == '__main__':
    main()