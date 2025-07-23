import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import random

class CircleTurtle(Node):
    def __init__(self):
        super().__init__('circle_turtle')

        self.declare_parameter('speed', 2.0)
        self.declare_parameter('radius', 10.0)
        self.declare_parameter('noise_std_dev', 3.0)

        self.speed = self.get_parameter('speed').get_parameter_value().double_value
        self.radius = self.get_parameter('radius').get_parameter_value().double_value
        self.noise_std_dev = self.get_parameter('noise_std_dev').get_parameter_value().double_value

        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.real_pose_pub = self.create_publisher(Pose, 'rt_real_pose', 10)
        self.noisy_pose_pub = self.create_publisher(Pose, 'rt_noisy_pose', 10)

        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        self.timer = self.create_timer(5.0, self.publish_poses)

        self.move_timer = self.create_timer(0.1, self.move_turtle)

        self.current_pose = Pose()

    def pose_callback(self, msg):

        self.current_pose = msg

    def move_turtle(self):

        twist_msg = Twist()
        twist_msg.linear.x = self.speed
        twist_msg.angular.z = self.speed / self.radius
        self.cmd_vel_pub.publish(twist_msg)

    def publish_poses(self):
 
        self.real_pose_pub.publish(self.current_pose)

        noisy_pose = Pose()
        noisy_pose.x = self.current_pose.x + random.gauss(0, self.noise_std_dev)
        noisy_pose.y = self.current_pose.y + random.gauss(0, self.noise_std_dev)
        noisy_pose.theta = self.current_pose.theta + random.gauss(0, self.noise_std_dev)

        self.noisy_pose_pub.publish(noisy_pose)

def main(args=None):
    rclpy.init(args=args)
    circle_turtle = CircleTurtle()
    rclpy.spin(circle_turtle)
    circle_turtle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
