import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math
import matplotlib.pyplot as plt

class TurtlePIDController(Node):
    def __init__(self):
        super().__init__('turtle_pid_controller')
        
        self.kp = 1.0
        self.ki = 1.0
        self.kd = 0.5
        
        self.goal_x = 30.0
        self.goal_y = 15.0
        
        self.error_integral = 0.0
        self.previous_error = 0.0

        self.timestamps = []
        self.distances_to_goal = []
        self.angle_errors = []
        self.linear_velocities = []
        self.angular_velocities = []
        self.x_positions = []
        self.y_positions = []

        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.time_step = 0 
    
    def pose_callback(self, msg: Pose):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_theta = msg.theta

    def control_loop(self):

        error_x = self.goal_x - self.current_x
        error_y = self.goal_y - self.current_y
        
        distance_to_goal = math.sqrt(error_x ** 2 + error_y ** 2)
        
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
        
        self.timestamps.append(self.time_step)
        self.distances_to_goal.append(distance_to_goal)
        self.angle_errors.append(angle_error)
        self.linear_velocities.append(linear_velocity)
        self.angular_velocities.append(angular_velocity)
        self.x_positions.append(self.current_x)
        self.y_positions.append(self.current_y)
        
        self.time_step += 1
        
        if distance_to_goal < 0.1:
            self.stop_turtle()
            self.plot_results()
    
    def stop_turtle(self):

        twist = Twist()
        self.cmd_pub.publish(twist)

    def normalize_angle(self, angle):

        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def plot_results(self):

        plt.figure(figsize=(12, 8))
        
        plt.subplot(2, 2, 1)
        plt.plot(self.timestamps, self.x_positions, label="x-position")
        plt.plot(self.timestamps, self.y_positions, label="y-position")
        plt.xlabel("Time Step")
        plt.ylabel("Position")
        plt.title("Turtle Position Over Time")
        plt.legend()
        
        plt.subplot(2, 2, 2)
        plt.plot(self.timestamps, self.distances_to_goal)
        plt.xlabel("Time Step")
        plt.ylabel("Distance to Goal")
        plt.title("Distance to Goal Over Time")
        
        plt.subplot(2, 2, 3)
        plt.plot(self.timestamps, self.angle_errors)
        plt.xlabel("Time Step")
        plt.ylabel("Angle Error")
        plt.title("Angle Error Over Time")
        
        plt.subplot(2, 2, 4)
        plt.plot(self.timestamps, self.linear_velocities, label="Linear Velocity")
        plt.plot(self.timestamps, self.angular_velocities, label="Angular Velocity")
        plt.xlabel("Time Step")
        plt.ylabel("Velocity")
        plt.title("Control Inputs Over Time")
        plt.legend()
        
        plt.tight_layout()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = TurtlePIDController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
