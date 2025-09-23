import rclpy
from rclpy.node import Node
from mirte_msgs.srv import SetSpeedMultiple
from mirte_msgs.msg import SetSpeedNamed
from std_msgs.msg import Int32
import signal
import random
import time
import math

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

class RichExcitationPWMController(Node):
    def __init__(self):
        super().__init__('excitation_pwm_generation')

        self.duration = 120.0
        self.interval = 0.3 # Time per pwm change - lower value = smoother ramps
        self.target_update_interval = 1.0 # New target

        self.max_delta_pwm = 20 # Max change per interval per wheel
        self.max_pwm = 100

        self.start_time = time.time()
        self.last_target_time = self.start_time
        self.phase = "random"
        self.pattern_index = 0

        # Current command and target (RF, RR, LF, LR)
        self.cmd = [0, 0, 0, 0]
        self.target = [0, 0, 0, 0]

        # Setup
        self.set_speed_multiple_client = self.create_client(
            SetSpeedMultiple, '/io/motor/motorservocontroller/set_multiple_speeds')
        # For logging data only
        self.left_front_pub = self.create_publisher(Int32, '/wheel_pwm/left_front', 10)
        self.right_front_pub = self.create_publisher(Int32, '/wheel_pwm/right_front', 10)
        self.left_rear_pub = self.create_publisher(Int32, '/wheel_pwm/left_rear', 10)
        self.right_rear_pub = self.create_publisher(Int32, '/wheel_pwm/right_rear', 10)

        while not self.set_speed_multiple_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for SetSpeedMultiple service...')

        signal.signal(signal.SIGINT, self.handle_shutdown)
        self.get_logger().info("Starting PWM excitation with slew limiting...")
        self.timer = self.create_timer(self.interval, self.pwm_gen)

    # Target generators
    def _random_template_target(self):
        # Separate into groups for equal excitation
        x_excitation = [
            lambda v: [ v,  v,  v,  v],   # forward
            lambda v: [-v, -v, -v, -v],   # backward
        ]
        rotate_excitation = [
            lambda v: [ v,  v, -v, -v],   # rotate left/right (x also excited a bit)
            lambda v: [-v, -v,  v,  v],
        ]
        y_excitation = [
            lambda v: [ v, -v, -v,  v],   # left
            lambda v: [-v,  v,  v, -v],   # right
            lambda v: [ v,  0,  0,  v],   # diag left forward
            lambda v: [ 0,  v,  v,  0],   # diag right forward
        ]

        # Alternate between groups each time for balance
        if random.random() < 0.3:
            template = random.choice(x_excitation)
        elif random.random() < 0.8:
            template = random.choice(y_excitation)
        else:
            template = random.choice(rotate_excitation)
        magnitude = random.randint(30, 100)
        return [clamp(x, -self.max_pwm, self.max_pwm) for x in template(magnitude)]


    def _pattern_target(self):
        patterns = [
            [-50, -50,  50,  50],  # spin right
            [ 50,  50, -50, -50],  # spin left
            [ 30,  40,  10,  30],  # arc left
            [ 40,  30,  30,  10],  # arc right
            [  0,  50,  50,   0],  # sharp right
            [ 50,   0,   0,  50],  # sharp left
            [ 40,  40,  40,  40],  # forward
            [-40, -40, -40, -40],  # backward
            [ 50,   0,   0,  50],  # diag LF
            [  0,  50,  50,   0],  # diag RF
        ]
        pwm = patterns[self.pattern_index % len(patterns)]
        self.pattern_index += 1
        return pwm

    def pwm_gen(self):
        now = time.time()
        elapsed = now - self.start_time

        if elapsed >= self.duration:
            self.get_logger().info("Duration complete. Stopping.")
            self.stop_wheels()
            self.destroy_timer(self.timer)
            return

        # Change to pattern at 60s
        self.phase = "random" if elapsed < 60.0 else "pattern"

        # Update target
        if (now - self.last_target_time) >= self.target_update_interval:
            if self.phase == "random":
                self.target = self._random_template_target()
                self.get_logger().info(f"[Target-Rand] RF={self.target[0]}, RR={self.target[1]}, LF={self.target[2]}, LR={self.target[3]}")
            else:
                self.target = self._pattern_target()
                self.get_logger().info(f"[Target-Pattern] RF={self.target[0]}, RR={self.target[1]}, LF={self.target[2]}, LR={self.target[3]}")
            self.last_target_time = now

        # Slew-rate limit
        new_cmd = []
        for i in range(4):
            diff = self.target[i] - self.cmd[i]
            step = clamp(diff, -self.max_delta_pwm, self.max_delta_pwm)
            new_cmd.append(clamp(self.cmd[i] + step, -self.max_pwm, self.max_pwm))

        # Only publish if something changed
        if new_cmd != self.cmd:
            self.cmd = new_cmd
            rf, rr, lf, lr = self.cmd
            self.set_wheel_speeds(rf, rr, lf, lr)
            self.get_logger().info(f"[Cmd Slewed] RF={rf}, RR={rr}, LF={lf}, LR={lr}")

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
        if future.result() is None or not future.result().success:
            self.get_logger().warn("Failed to send PWM command.")

    def stop_wheels(self):
        self.cmd = [0, 0, 0, 0]
        self.target = [0, 0, 0, 0]
        self.set_wheel_speeds(0, 0, 0, 0)

    def handle_shutdown(self, signum, frame):
        self.get_logger().warn("Ctrl+C detected. Stopping wheels.")
        self.stop_wheels()
        rclpy.shutdown()

def main():
    rclpy.init()
    node = RichExcitationPWMController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("Ctrl+C detected. Stopping wheels.")
        node.stop_wheels()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
