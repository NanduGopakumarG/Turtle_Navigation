import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class TurtleController(Node):
    def __init__(self):
        super().__init__("turtle_controller")

        # Goal position (updated dynamically for grid)
        self.goal_x = 1.0
        self.goal_y = 1.0

        # PID gains
        self.kp_lin = 1.0
        self.ki_lin = 0.0
        self.kd_lin = 0.5

        self.kp_ang = 1.0
        self.ki_ang = 0.0
        self.kd_ang = 0.5

        # Errors
        self.prev_error_lin = 0.0
        self.prev_error_ang = 0.0
        self.integral_lin = 0.0
        self.integral_ang = 0.0

        # Velocity limits
        self.max_speed = 2.0  # Maximum velocity (m/s)
        self.max_acc = 0.5    # Maximum acceleration (m/s²)
        self.max_dec = 0.2    # Maximum deceleration (m/s²)
        self.current_speed = 0.0  # Track speed over time

        # ROS subscribers and publishers
        self.pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.pose = None

    def pose_callback(self, msg):
        """ Updates the turtle's current position """
        self.pose = msg

    def control_loop(self):
        """ PID-based velocity control with acceleration limits """
        if self.pose is None:
            return

        # Compute errors
        error_lin = math.sqrt((self.goal_x - self.pose.x) ** 2 + (self.goal_y - self.pose.y) ** 2)
        target_angle = math.atan2(self.goal_y - self.pose.y, self.goal_x - self.pose.x)
        error_ang = target_angle - self.pose.theta

        # PID calculations
        self.integral_lin += error_lin * 0.1
        self.integral_ang += error_ang * 0.1

        derivative_lin = (error_lin - self.prev_error_lin) / 0.1
        derivative_ang = (error_ang - self.prev_error_ang) / 0.1

        control_lin = (self.kp_lin * error_lin) + (self.ki_lin * self.integral_lin) + (self.kd_lin * derivative_lin)
        control_ang = (self.kp_ang * error_ang) + (self.ki_ang * self.integral_ang) + (self.kd_ang * derivative_ang)

        # Update previous errors
        self.prev_error_lin = error_lin
        self.prev_error_ang = error_ang

        # Apply acceleration/deceleration constraints
        if control_lin > self.current_speed + self.max_acc * 0.1:
            self.current_speed += self.max_acc * 0.1
        elif control_lin < self.current_speed - self.max_dec * 0.1:
            self.current_speed -= self.max_dec * 0.1
        else:
            self.current_speed = control_lin

        # Limit speed
        self.current_speed = max(0.0, min(self.current_speed, self.max_speed))

        # Command velocity
        cmd = Twist()
        cmd.linear.x = self.current_speed  # Smoothed speed
        cmd.angular.z = control_ang

        self.vel_pub.publish(cmd)

        # Stop condition
        if error_lin < 0.1:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.vel_pub.publish(cmd)
            self.get_logger().info("Goal reached!")

def main():
    rclpy.init()
    node = TurtleController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
