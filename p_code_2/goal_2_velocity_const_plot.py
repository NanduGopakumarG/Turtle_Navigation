import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math
import matplotlib.pyplot as plt

class TurtlePIDController(Node):
    def __init__(self):
        super().__init__('turtle_pid_controller')

        self.goal_x = 1.0
        self.goal_y = 1.0

        self.kp_ang = 1.5
        self.ki_ang = 1.0
        self.kd_ang = 0.5

        self.kp_lin = 2.5
        self.ki_lin = 1.0
        self.kd_lin = 0.5

        self.prev_error_ang = 0.0
        self.integral_ang = 0.0
        self.prev_error_lin = 0.0
        self.integral_lin = 0.0

        self.max_speed = 4.0  
        self.max_acc = 1.0
        self.max_dec = 0.1
        self.current_speed = 0.0

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        self.pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.timestamps = []
        self.distances_to_goal = []
        self.angle_errors = []
        self.linear_velocities = []
        self.angular_velocities = []
        self.x_positions = []
        self.y_positions = []
        self.time_step = 0

    def pose_callback(self, msg: Pose):

        self.current_x = msg.x
        self.current_y = msg.y
        self.current_theta = msg.theta

    def control_loop(self):
   
        if self.current_x == 0.0 and self.current_y == 0.0:
            return 
 
        error_x = self.goal_x - self.current_x
        error_y = self.goal_y - self.current_y
        distance_to_goal = math.sqrt(error_x ** 2 + error_y ** 2)

        goal_angle = math.atan2(error_y, error_x)
        angle_error = self.normalize_angle(goal_angle - self.current_theta)

        self.integral_ang += angle_error * 0.1
        derivative_ang = (angle_error - self.prev_error_ang) / 0.1
        angular_velocity = (
            self.kp_ang * angle_error +
            self.ki_ang * self.integral_ang +
            self.kd_ang * derivative_ang
        )
        self.prev_error_ang = angle_error

        if abs(angle_error) < 0.2: 
            self.integral_lin += distance_to_goal * 0.1
            derivative_lin = (distance_to_goal - self.prev_error_lin) / 0.1
            linear_velocity = (
                self.kp_lin * distance_to_goal +
                self.ki_lin * self.integral_lin +
                self.kd_lin * derivative_lin
            )
            self.prev_error_lin = distance_to_goal
        else:
            linear_velocity = 0.0 

        if linear_velocity > self.current_speed + self.max_acc * 0.1:
            self.current_speed += self.max_acc * 0.1
        elif linear_velocity < self.current_speed - self.max_dec * 0.1:
            self.current_speed -= self.max_dec * 0.1
        else:
            self.current_speed = linear_velocity

        self.current_speed = max(0.0, min(self.current_speed, self.max_speed))

        twist = Twist()
        twist.linear.x = self.current_speed
        twist.angular.z = angular_velocity
        self.vel_pub.publish(twist)

        self.timestamps.append(self.time_step)
        self.distances_to_goal.append(distance_to_goal)
        self.angle_errors.append(angle_error)
        self.linear_velocities.append(self.current_speed)
        self.angular_velocities.append(angular_velocity)
        self.x_positions.append(self.current_x)
        self.y_positions.append(self.current_y)

        self.time_step += 1

        if distance_to_goal < 0.1:
            self.stop_turtle()
            self.plot_results()

    def stop_turtle(self):

        twist = Twist()
        self.vel_pub.publish(twist)
        self.get_logger().info("Goal reached!")

    def normalize_angle(self, angle):

        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def plot_results(self):

        plt.figure(figsize=(12, 8))

        plt.subplot(2, 2, 1)
        plt.plot(self.x_positions, self.y_positions, marker='o', linestyle='-')
        plt.scatter(self.goal_x, self.goal_y, color='red', label="Goal")
        plt.xlabel("X Position")
        plt.ylabel("Y Position")
        plt.title("Turtle Position Trajectory")
        plt.legend()

        plt.subplot(2, 2, 2)
        plt.plot(self.timestamps, self.distances_to_goal)
        plt.xlabel("Time Step")
        plt.ylabel("Distance to Goal")
        plt.title("Distance to Goal Over Time")

        plt.subplot(2, 2, 3)
        plt.plot(self.timestamps, self.angle_errors)
        plt.xlabel("Time Step")
        plt.ylabel("Angle Error (radians)")
        plt.title("Angle Error Over Time")

        plt.subplot(2, 2, 4)
        plt.plot(self.timestamps, self.linear_velocities, label="Linear Velocity")
        plt.plot(self.timestamps, self.angular_velocities, label="Angular Velocity")
        plt.xlabel("Time Step")
        plt.ylabel("Velocity (m/s)")
        plt.title("Control Inputs Over Time")
        plt.legend()

        plt.tight_layout()
        plt.show()

def main():
    rclpy.init()
    node = TurtlePIDController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

