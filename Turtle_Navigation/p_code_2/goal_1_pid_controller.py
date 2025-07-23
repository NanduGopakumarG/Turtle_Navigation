import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math

class TurtlePIDController(Node):
    def __init__(self):
        super().__init__('turtle_pid_controller')
        
        self.kp = 1.0
        self.ki = 0.0
        self.kd = 0.5

        self.goal_x = 15.0
        self.goal_y = 12.0

        self.error_integral = 0.0
        self.previous_error = 0.0

        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.control_loop)

    def pose_callback(self, msg: Pose):

        self.current_x = msg.x
        self.current_y = msg.y
        self.current_theta = msg.theta

    def control_loop(self):

        error_x = self.goal_x - self.current_x
        error_y = self.goal_y - self.current_y

        distance_to_goal = math.sqrt(error_x ** 2 + error_y ** 2)

        if distance_to_goal < 0.1:
            self.get_logger().info('Goal reached, stopping turtle.')
            self.stop_turtle()
            return

        goal_angle = math.atan2(error_y, error_x)

        angle_error = self.normalize_angle(goal_angle - self.current_theta)

        self.error_integral += angle_error * 0.1 
        error_derivative = (angle_error - self.previous_error) / 0.1 

        angular_velocity = self.kp * angle_error + self.ki * self.error_integral + self.kd * error_derivative

        linear_velocity = min(1.5 * distance_to_goal, 2.0) 

        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity

        self.cmd_pub.publish(twist)

        self.previous_error = angle_error
        
    def stop_turtle(self):

        twist = Twist()
        self.cmd_pub.publish(twist)

    def normalize_angle(self, angle):

        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = TurtlePIDController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
