import struct
import math
import serial
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# custom math
from foo import RoverOdometry

class EncoderReaderNode(Node):
    def __init__(self):
        super().__init__('encoder_reader_node')

        # 1. Initialize Math Tracker from foo.py (adjust parameters to match your rover)
        self.odom_tracker = RoverOdometry(wheel_radius=0.06, wheel_base=0.47, ticks_per_rev=90)
        self.get_logger().info(f"Odom -> X: {x:.2f}m | Y: {y:.2f}m | Theta: {math.degrees(theta):.1f}°")
        
        # 2. Setup ROS 2 Publishers and TF Broadcasters
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Configure Serial Port
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
            self.get_logger().info("Connected to Arduino over Serial.")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.ser = None

        # Check for serial data at 25 Hz (every 0.04s) ,50hz idealy
        self.timer = self.create_timer(0.04, self.read_encoder_data)

    def read_encoder_data(self):
        if not self.ser or not self.ser.is_open:
            return

        # Ensure we have at least 9 bytes (1 full packet) in the buffer
        if self.ser.in_waiting >= 9:
            # Look for Header Byte (0xFF)
            header_byte = self.ser.read(1)
            
            if header_byte == b'\xff':
                # Read the remaining 8 data bytes
                data_bytes = self.ser.read(8)
                
                if len(data_bytes) == 8:
                    # Unpack two 32-bit signed integers (<ii)
                    left_ticks, right_ticks = struct.unpack('<ii', data_bytes)
                    self.get_logger().info(f"Encoders -> Left: {left_ticks} | Right: {right_ticks}")
                    ## math ##
                    # Get ROS 2 system clock
                    now = self.get_clock().now()
                    time_sec = now.nanoseconds / 1e9

                    # --- A. Compute Odometry with foo.py ---
                    x, y, theta, v, w = self.odom_tracker.update(left_ticks, right_ticks, time_sec)

                    # --- B. Convert Euler Theta to Quaternion ---
                    qz = math.sin(theta / 2.0)
                    qw = math.cos(theta / 2.0)

                    # --- C. Broadcast TF Transform (odom -> base_link) ---
                    t = TransformStamped()
                    t.header.stamp = now.to_msg()
                    t.header.frame_id = 'odom'
                    t.child_frame_id = 'base_link'
                    t.transform.translation.x = x
                    t.transform.translation.y = y
                    t.transform.translation.z = 0.0
                    t.transform.rotation.z = qz
                    t.transform.rotation.w = qw
                    self.tf_broadcaster.sendTransform(t)

                    # --- D. Publish /odom Topic Message ---
                    odom_msg = Odometry()
                    odom_msg.header.stamp = now.to_msg()
                    odom_msg.header.frame_id = 'odom'
                    odom_msg.child_frame_id = 'base_link'
                    
                    odom_msg.pose.pose.position.x = x
                    odom_msg.pose.pose.position.y = y
                    odom_msg.pose.pose.orientation.z = qz
                    odom_msg.pose.pose.orientation.w = qw
                    
                    odom_msg.twist.twist.linear.x = v
                    odom_msg.twist.twist.angular.z = w

                    self.odom_pub.publish(odom_msg)

            else:
                # Sink misaligned header byte
                pass

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = EncoderReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
