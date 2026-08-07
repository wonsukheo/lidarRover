import math

# ===============================================================
# 1. SERVO CONVERSION FUNCTIONS (Joystick & Nav2 Control)
# ===============================================================

def convert_to_servo(speed_ratio: float, steer_ratio: float, max_range: float = 45.0, deadband: float = 0.05) -> tuple[int, int]:
    """
    Converts normalized throttle (-1.0 to 1.0) and steering (-1.0 to 1.0)
    into Servo values (0 to 180, where 90 is stop/neutral).
    """
    # deadband
    if abs(speed_ratio) < deadband:
        speed_ratio = 0.0
    if abs(steer_ratio) < deadband:
        steer_ratio = 0.0

    target_left = 90.0 + (speed_ratio * 90.0) - (steer_ratio * 90.0)
    target_right = 90.0 + (speed_ratio * 90.0) + (steer_ratio * 90.0)
    
    left_motor = int(max(45, min(135, target_left)))
    right_motor = int(max(45, min(135, target_right)))

    return left_motor, right_motor


def process_cmd_vel(linear_x: float, angular_z: float, max_linear: float = 0.5, max_angular: float = 1.0) -> tuple[int, int]:
    """
    Normalizes Nav2 Twist velocity messages (/cmd_vel) and converts them to Servo values.
    """
    norm_speed = linear_x / max_linear
    norm_steer = angular_z / max_angular

    return convert_to_servo(norm_speed, norm_steer)


# ===============================================================
# 2. ODOMETRY TRACKER CLASS (Encoder Tick Math)
# ===============================================================

class RoverOdometry:
    def __init__(self, wheel_radius: float = 0.12, wheel_base: float = 0.47, ticks_per_rev: int = 90):
        """
        Tracks differential drive robot pose (x, y, theta) and velocities (v, w)
        based on raw encoder tick feedback.
        """
        self.wheel_radius = wheel_radius        # Wheel radius in meters
        self.wheel_base = wheel_base            # Distance between left & right wheels in meters
        self.ticks_per_rev = ticks_per_rev      # Encoder pulses per full wheel revolution
        
        # Global Pose State
        self.x = 0.0                            # Position X (meters)
        self.y = 0.0                            # Position Y (meters)
        self.theta = 0.0                        # Heading angle (radians)
        
        # Historical state tracker
        self.last_left_ticks = None
        self.last_right_ticks = None
        self.last_time = None

    def update(self, current_left_ticks: int, current_right_ticks: int, current_time_sec: float) -> tuple[float, float, float, float, float]:
        """
        Updates robot pose based on current raw encoder ticks and timestamp.
        Returns: (x, y, theta, linear_v, angular_w)
        """
        # First-time initialization
        if self.last_left_ticks is None:
            self.last_left_ticks = current_left_ticks
            self.last_right_ticks = current_right_ticks
            self.last_time = current_time_sec
            return self.x, self.y, self.theta, 0.0, 0.0

        # Calculate time delta (dt)
        dt = current_time_sec - self.last_time
        if dt <= 0.0:
            return self.x, self.y, self.theta, 0.0, 0.0

        # Calculate tick changes
        delta_left = current_left_ticks - self.last_left_ticks
        delta_right = current_right_ticks - self.last_right_ticks

        self.last_left_ticks = current_left_ticks
        self.last_right_ticks = current_right_ticks
        self.last_time = current_time_sec

        # Convert tick delta to linear distance moved per wheel (meters)
        meters_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev
        d_left = delta_left * meters_per_tick
        d_right = delta_right * meters_per_tick

        # Differential drive kinematics
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.wheel_base

        # Update pose (midpoint integration)
        if d_center != 0.0:
            self.x += d_center * math.cos(self.theta + (d_theta / 2.0))
            self.y += d_center * math.sin(self.theta + (d_theta / 2.0))
        
        self.theta += d_theta
        
        # Normalize theta between -pi and pi
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Calculate linear and angular velocities
        linear_v = d_center / dt
        angular_w = d_theta / dt

        return self.x, self.y, self.theta, linear_v, angular_w
        