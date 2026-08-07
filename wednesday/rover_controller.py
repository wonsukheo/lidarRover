import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class RoverControllerNode(Node):
    def __init__(self):
        super().__init__('rover_controller_node')

        # ROS2 subscription: joystick manual and Navigation2 autonomous velocity data
        self.sub_joy = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Serial connection to Arduino via USB
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
            self.get_logger().info("Connected to Arduino over Serial.")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.ser = None

        # --- System State ---
        # 0 = Manual (Joystick) | 1 = Autonomous (Nav2)
        self.mode = 0  
        
        # Stored motor values (90 = Neutral / Rest)
        self.manual_left = 90
        self.manual_right = 90
        self.nav_left = 90
        self.nav_right = 90

        # Debounce state for controller button mode toggle
        self.last_button_state = 0

        # --- Main Loop Timer (50 Hz / 0.02s) ---
        self.timer = self.create_timer(0.02, self.send_serial_packet)

    # ===============================================================
    # 1. HELPER CONVERSION FUNCTION
    # Ps5 joystick inputs (-1.0 to 1.0) -> outputs Servo values (0 to 180)
    # ===============================================================
    def convert_to_servo(self, speed_ratio: float, steer_ratio: float):
        target_left = 90 + (speed_ratio * 90.0) + (steer_ratio * 90.0)
        target_right = 90 + (speed_ratio * 90.0) - (steer_ratio * 90.0)
        
        left_motor = int(max(0, min(180, target_left)))
        right_motor = int(max(0, min(180, target_right)))

        return left_motor, right_motor
    # ===============================================================
    # 2. MANUAL CALLBACK (Joystick Input)
    # ===============================================================
    def joy_callback(self, msg):
        # Toggle mode when Button 0 ('A') is pressed
        button_state = msg.buttons[0]

        if button_state == 1 and self.last_button_state == 0:
            self.mode = 1 if self.mode == 0 else 0
            # debug
            mode_name = "AUTONOMOUS (Nav2)" if self.mode == 1 else "MANUAL (Joystick)"
            self.get_logger().info(f"Mode Switched -> {mode_name}")

        self.last_button_state = button_state

        # Calculate manual servo values
        steering = msg.axes[0]
        throttle = msg.axes[1]
        self.manual_left, self.manual_right = self.convert_to_servo(throttle, steering)
    # ===============================================================
    # 3. AUTONOMOUS CALLBACK (Nav2 Input)
    # ===============================================================
    def cmd_vel_callback(self, msg):
        MAX_LINEAR_SPEED = 0.5   # Max forward velocity in m/s
        MAX_ANGULAR_SPEED = 1.0  # Max turning velocity in rad/s

        # Normalize Nav2 speed values to the -1.0 ~ 1.0 range
        norm_speed = msg.linear.x / MAX_LINEAR_SPEED
        norm_steer = msg.angular.z / MAX_ANGULAR_SPEED

        # Calculate autonomous servo values
        self.nav_left, self.nav_right = self.convert_to_servo(norm_speed, norm_steer)
    # ===============================================================
    # 4. MAIN SERIAL SENDER (Kept minimal and fast)
    # ===============================================================
    def send_serial_packet(self):
        if not self.ser or not self.ser.is_open:
            return

        # Clear incoming serial buffer to prevent latency/overflow
        self.ser.reset_input_buffer()

        # Select values based on current active mode
        if self.mode == 0:
            left, right = self.manual_left, self.manual_right
        else:
            left, right = self.nav_left, self.nav_right

        # Send 3-byte binary packet [0xFF, Left, Right]
        packet = bytes([0xFF, left, right])
        self.ser.write(packet)
    # ===============================================================
    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RoverControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()