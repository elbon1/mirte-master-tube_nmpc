import rclpy
from rclpy.node import Node
from mirte_msgs.srv import SetSpeedMultiple
from mirte_msgs.msg import SetSpeedNamed
from std_msgs.msg import Int32
import signal  # Handle Ctrl+C
import math

class WheelSpeedController(Node):
    def __init__(self):
        super().__init__('wheel_speed_controller')
        self.timer = None
        self.state = 0  # Track movement state

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

        # Handle Ctrl+C
        signal.signal(signal.SIGINT, self.handle_shutdown)

    def handle_shutdown(self, signum, frame):
        # Immediately stop robot on Ctrl+C
        self.get_logger().warn("Emergency stop! Stopping wheels...")
        self.stop_wheels()
        rclpy.shutdown()

    def set_wheel_speeds(self, right_front, right_rear, left_front, left_rear):
        # Send wheel speed commands to the robot
        
        self.right_front_pub.publish(Int32(data=right_front))
        self.right_rear_pub.publish(Int32(data=right_rear))
        self.left_front_pub.publish(Int32(data=left_front))
        self.left_rear_pub.publish(Int32(data=left_rear))

        request = SetSpeedMultiple.Request()
        request.speeds = [
            SetSpeedNamed(speed=right_front, name="front_right"),
            SetSpeedNamed(speed=right_rear, name="rear_right"),
            SetSpeedNamed(speed=left_front, name="front_left"),
            SetSpeedNamed(speed=left_rear, name="rear_left")
        ]

        # Call the service asynchronously
        future = self.set_speed_multiple_client.call_async(request)
        future.add_done_callback(self.speed_service_response)

    def speed_service_response(self, future):
        # Check if the service call was successful
        if future.result() is not None and future.result().success:
            self.get_logger().info('Successfully set wheel speeds.')
        else:
            self.get_logger().warn('Failed to set wheel speeds.')

    def stop_wheels(self):
        # Stop all wheels
        self.set_wheel_speeds(0, 0, 0, 0)

    def move_forward(self):
        self.set_wheel_speeds(50, 50, 50, 50)

    def move_backward(self):
        self.set_wheel_speeds(-50, -50, -50, -50)
    
    def turn_right(self):
        self.set_wheel_speeds(-30, -30, 30, 30)

    def turn_left(self):
        self.set_wheel_speeds(30, 30, -30, -30)

    def move_right(self):
        self.set_wheel_speeds(-50, 50, 50, -50)

    def move_left(self):
        self.set_wheel_speeds(50, -50, -50, 50)

    def move_forwardleft(self):
        self.set_wheel_speeds(50, 0, 0, 50)

    def move_forwardright(self):
        self.set_wheel_speeds(0, 50, 50, 0)

    def move_backwardleft(self):
        self.set_wheel_speeds(0, -50, -50, 0)

    def move_backwardright(self):
        self.set_wheel_speeds(-50, 0, 0, -50)

    def lateral_arc(self):
        self.set_wheel_speeds(-50, 0, 50, 0)

    def move_trajectory(self):
        self.get_logger().info("Starting trajectory...")
        self.state = 0  # Reset state
        self.execute_next_movement()

    def execute_next_movement(self):
        delay_move = 4.0
        delay_turn = 2.0

        # Stop any existing timer before setting a new one
        if self.timer is not None:
            self.destroy_timer(self.timer)
            self.timer = None
        
        '''
        # move forward
        if self.state == 0:
            self.move_forward()
            delay = 5.0
            self.state = 1
        elif self.state == 1:
            self.stop_wheels()
            self.get_logger().info("Trajectory complete.")
            return  # Stop execution
        
        # rectangle with no turning
        if self.state == 0:
            self.move_forward()
            delay = delay_move
            self.state = 1
        elif self.state == 1:
            self.move_right()
            delay = delay_turn
            self.state = 2
        elif self.state == 2:
            self.move_backward()
            delay = delay_move
            self.state = 3
        elif self.state == 3:
            self.move_left()
            delay = delay_turn
            self.state = 4
        elif self.state == 4:
            self.stop_wheels()
            self.get_logger().info("Trajectory complete.")
            return  # Stop execution
        '''
        # rectangle with turn
        if self.state == 0:
            self.move_forward()
            delay = delay_move
            self.state = 1
        elif self.state == 1:
            self.turn_right()
            delay = delay_turn
            self.state = 2
        elif self.state == 2:
            self.move_forward()
            delay = delay_move
            self.state = 3
        elif self.state == 3:
            self.turn_right()
            delay = delay_turn
            self.state = 4
        elif self.state == 4:
            self.move_forward()
            delay = delay_move
            self.state = 5
        elif self.state == 5:
            self.turn_right()
            delay = delay_turn
            self.state = 6
        elif self.state == 6:
            self.move_forward()
            delay = delay_move
            self.state = 7
        elif self.state == 7:
            self.stop_wheels()
            self.get_logger().info("Trajectory complete.")
            return
        
        # Restart the timer with the new delay
        self.timer = self.create_timer(delay, self.execute_next_movement)

    def move_circle_fixed_heading(self):

        self.duration=16.0
        self.speed=50
        self.update_rate=2.0 # Time (s) for updating wheel speeds
        
        self.get_logger().info("Starting circular trajectory with fixed heading...")

        self.radius = 0.8
        self.omega = 2 * math.pi / self.duration  # Angular velocity (rad/s)
        self.time_elapsed = 0.0  # Track movement time
        
        # Start a timer to update wheel speeds at every interval
        self.timer = self.create_timer(self.update_rate, self.update_wheel_speeds_fixed)

    
    def update_wheel_speeds_fixed(self):
    
        if self.time_elapsed >= self.duration:
            self.stop_wheels()
            self.destroy_timer(self.timer)
            return

        if self.state == 0:
            self.move_forward()
            delay = self.update_rate
            self.state = 1
        elif self.state == 1:
            self.move_forwardright()
            delay = self.update_rate
            self.state = 2
        elif self.state == 2:
            self.move_right()
            delay = self.update_rate
            self.state = 3
        elif self.state == 3:
            self.move_backwardright()
            delay = self.update_rate
            self.state = 4
        elif self.state == 4:
            self.move_backward()
            delay = self.update_rate
            self.state = 5
        elif self.state == 5:
            self.move_backwardleft()
            delay = self.update_rate
            self.state = 6
        elif self.state == 6:
            self.move_left()
            delay = self.update_rate
            self.state = 7
        elif self.state == 7:
            self.move_forwardleft()
            delay = self.update_rate
            self.state = 8
        elif self.state == 8:
            self.stop_wheels()
            self.get_logger().info("Trajectory complete.")
            return  # Stop execution

        # Increment time
        self.time_elapsed += self.update_rate

    def move_circle_varying_heading(self):
    

        self.duration=18 
        self.update_rate=0.1 # Time (s) for updating wheel speeds

        self.get_logger().info("Starting circular trajectory with varying heading...")

        self.time_elapsed = 0.0  # Track movement time
    
        # Start a timer to update wheel speeds at every interval
        self.timer = self.create_timer(self.update_rate, self.update_wheel_speeds)
        
    def update_wheel_speeds(self):
    
        if self.time_elapsed >= self.duration:
            self.stop_wheels()
            self.destroy_timer(self.timer)
            return

        # Apply speeds to the wheels
        self.set_wheel_speeds(50, 50, 30, 30)

        # Increment time
        self.time_elapsed += self.update_rate

def main():
    rclpy.init()
    wheel_controller = WheelSpeedController()

    try:
        wheel_controller.move_trajectory()
        #wheel_controller.move_circle_fixed_heading()
        #wheel_controller.move_circle_varying_heading()
        rclpy.spin(wheel_controller)
    except KeyboardInterrupt:
        wheel_controller.get_logger().warn('Keyboard Interrupt detected! Stopping wheels...')
        wheel_controller.stop_wheels()
    finally:
        wheel_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()