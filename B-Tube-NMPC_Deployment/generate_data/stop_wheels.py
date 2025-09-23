import rclpy
from rclpy.node import Node
from mirte_msgs.srv import SetSpeedMultiple
from mirte_msgs.msg import SetSpeedNamed
from std_msgs.msg import Int32
from time import sleep

class WheelSpeedController(Node):
    def __init__(self):
        super().__init__('wheel_speed_controller')

        # Create a client for SetSpeedMultiple service
        self.set_speed_multiple_client = self.create_client(SetSpeedMultiple, '/io/motor/motorservocontroller/set_multiple_speeds')

        # Create publishers for PWM values
        self.left_front_pub = self.create_publisher(Int32, '/wheel_pwm/left_front', 10)
        self.right_front_pub = self.create_publisher(Int32, '/wheel_pwm/right_front', 10)
        self.left_rear_pub = self.create_publisher(Int32, '/wheel_pwm/left_rear', 10)
        self.right_rear_pub = self.create_publisher(Int32, '/wheel_pwm/right_rear', 10)

        # Wait until the service is available
        while not self.set_speed_multiple_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for SetSpeedMultiple service...')

    def set_wheel_speeds(self, right_front, right_rear, left_front, left_rear):
        # Send wheel speed commands to the robot

        self.left_front_pub.publish(Int32(data=left_front))
        self.right_front_pub.publish(Int32(data=right_front))
        self.left_rear_pub.publish(Int32(data=left_rear))
        self.right_rear_pub.publish(Int32(data=right_rear))

        request = SetSpeedMultiple.Request()

        request.speeds = [
            SetSpeedNamed(speed=right_front, name="right_front"),
            SetSpeedNamed(speed=right_rear, name="right_rear"),
            SetSpeedNamed(speed=left_front, name="left_front"),
            SetSpeedNamed(speed=left_rear, name="left_rear")
        ]

        # Call the service asynchronously
        future = self.set_speed_multiple_client.call_async(request)

        # Wait for the result (success or failure)
        rclpy.spin_until_future_complete(self, future)

        # Check if the service call was successful
        if future.result() is not None and future.result().success:
            self.get_logger().info('Successfully set wheel speeds.')
        else:
            self.get_logger().warn('Failed to set wheel speeds.')
    
    def stop_wheels(self):
        # Stop all wheels
        self.set_wheel_speeds(0, 0, 0, 0)

def main():
    rclpy.init()
    wheel_controller = WheelSpeedController()  # Create WheelSpeedController instance

    try:
        wheel_controller.stop_wheels()

        # Keep the node alive
        rclpy.spin(wheel_controller)

    except KeyboardInterrupt:
        wheel_controller.get_logger().info('Keyboard Interrupt detected! Stopping wheels...')
        wheel_controller.stop_wheels()
    finally:
        wheel_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()