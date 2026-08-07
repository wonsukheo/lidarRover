# foo.py
import math

class RoverOdometry:
    def __init__(self, wheel_radius=12, wheel_base=47, ticks_per_rev=90):
        # Physical parameters of your rover (adjust to match your hardware)
        self.wheel_radius = wheel_radius        # Radius in meters
        self.wheel_base = wheel_base            # Distance between left and right wheels in meters
        self.ticks_per_rev = ticks_per_rev      # Encoder pulses per full revolution
        
        # State variables
        self.x = 0.0                            # Position x in meters
        self.y = 0.0                            # Position y in meters
        self.theta = 0.0                        # Heading angle in radians
        
        self.last_left_ticks = None
        self.last_right_ticks = None
        self.last_time = None

    def update(self, current_left_ticks: int, current_right_ticks: int, current_time_sec: float):
        """
        Takes raw encoder ticks and current timestamp.
        Returns: (x, y, theta, linear_v, angular_w)
        """
        # First-time initialization
        if self.last_left_ticks is None:
            self.last_left_ticks = current_left_ticks
            self.last_right_ticks = current_right_ticks
            self.last_time = current_time_sec
            return self.x, self.y, self.theta, 0.0, 0.0

        # 1. Calculate time delta (dt)
        dt = current_time_sec - self.last_time
        if dt <= 0.0:
            return self.x, self.y, self.theta, 0.0, 0.0

        # 2. Calculate change in ticks
        delta_left = current_left_ticks - self.last_left_ticks
        delta_right = current_right_ticks - self.last_right_ticks

        self.last_left_ticks = current_left_ticks
        self.last_right_ticks = current_right_ticks
        self.last_time = current_time_sec

        # 3. Convert tick changes to wheel displacement (meters)
        meters_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev
        d_left = delta_left * meters_per_tick
        d_right = delta_right * meters_per_tick

        # 4. Compute linear displacement (d) and angular displacement (d_theta)
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.wheel_base

        # 5. Update global pose (x, y, theta) using midpoint integration
        if d_center != 0.0:
            self.x += d_center * math.cos(self.theta + (d_theta / 2.0))
            self.y += d_center * math.sin(self.theta + (d_theta / 2.0))
        
        self.theta += d_theta
        # Keep theta normalized between -pi and pi
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # 6. Calculate speeds (m/s and rad/s)
        linear_v = d_center / dt
        angular_w = d_theta / dt

        return self.x, self.y, self.theta, linear_v, angular_w
    