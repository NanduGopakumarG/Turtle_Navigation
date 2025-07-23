import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error):

        self.integral += error
        derivative = error - self.prev_error
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        self.targets = [
            (1.0, 1.0, 0.0),
            (10.0, 1.0, math.radians(90)),
            (10.0, 2.0, math.radians(180)),
            (1.0, 2.0, math.radians(90)),
            (1.0, 3.0, 0.0),
            (10.0, 3.0, math.radians(90)),
            (10.0, 4.0, math.radians(180)),
            (1.0, 4.0, math.radians(90)),
            (1.0, 5.0, 0.0),
            (10.0, 5.0, 0.0)
        ]
        self.current_target_index = 0
        self.target_x, self.target_y, self.target_theta = self.targets[self.current_target_index]
        
        self.current_pose = None
        self.timer = self.create_timer(0.1, self.move_turtle)
        self.reached_position = False

        self.linear_pid = PIDController(Kp=1.5, Ki=0.0, Kd=0.4)  
        self.angular_pid = PIDController(Kp=4.0, Ki=0.0, Kd=0.3)

    def pose_callback(self, msg):
        self.current_pose = msg

    def move_turtle(self):
        if self.current_pose is None:
            return

        dx = self.target_x - self.current_pose.x
        dy = self.target_y - self.current_pose.y
        distance = math.sqrt(dx**2 + dy**2)

        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - self.current_pose.theta

        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

        twist = Twist()

        if not self.reached_position:
        
            linear_error = distance
            twist.linear.x = self.linear_pid.compute(linear_error)

            angular_error = angle_diff
            twist.angular.z = self.angular_pid.compute(angular_error)

            if distance < 0.05:
                twist.linear.x = 0.0
                self.reached_position = True
        else:

            angle_error = self.target_theta - self.current_pose.theta
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

            if abs(angle_error) > 0.01:
                twist.angular.z = self.angular_pid.compute(angle_error)
            else:
                twist.angular.z = 0.0
                self.get_logger().info(f"Target {self.current_target_index + 1} reached!")

                self.current_target_index += 1
                if self.current_target_index < len(self.targets):
                    self.target_x, self.target_y, self.target_theta = self.targets[self.current_target_index]
                    self.reached_position = False
                else:
                    self.get_logger().info("All targets reached!")
                    self.timer.cancel() 

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    turtle_controller = TurtleController()
    rclpy.spin(turtle_controller)
    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
