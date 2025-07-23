import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
import math
import random

class RunnerTurtle(Node):
    def __init__(self):
        super().__init__('runner_turtle')
        
        self.speed = 3.2  # RT Speed
        self.radius = 5.0  # RT Circular Path Radius
        self.noise_std_dev = 0.5  # Noise in RT's pose
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.noisy_pose_pub = self.create_publisher(Pose, 'rt_noisy_pose', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        
        self.timer = self.create_timer(5.0, self.publish_noisy_pose)
        self.move_timer = self.create_timer(0.1, self.move_turtle)
        
        self.current_pose = Pose()

    def pose_callback(self, msg):
        self.current_pose = msg

    def move_turtle(self):
        twist_msg = Twist()
        twist_msg.linear.x = self.speed
        twist_msg.angular.z = self.speed / self.radius
        self.cmd_vel_pub.publish(twist_msg)

    def publish_noisy_pose(self):
        noisy_pose = Pose()
        noisy_pose.x = self.current_pose.x + random.gauss(0, self.noise_std_dev)
        noisy_pose.y = self.current_pose.y + random.gauss(0, self.noise_std_dev)
        noisy_pose.theta = self.current_pose.theta + random.gauss(0, self.noise_std_dev)
        self.noisy_pose_pub.publish(noisy_pose)

class PoliceTurtle(Node):
    def __init__(self):
        super().__init__('police_turtle')
        
        self.pt_speed = 1.5  # PT Speed
        self.spawned = False
        self.rt_noisy_pose = Pose()
        self.police_pose = Pose()
        
        self.spawn_timer = self.create_timer(2.0, self.spawn_turtle)
        self.noisy_pose_sub = self.create_subscription(Pose, 'rt_noisy_pose', self.noisy_pose_callback, 10)
        self.police_pose_sub = self.create_subscription(Pose, '/police_turtle/pose', self.police_pose_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/police_turtle/cmd_vel', 10)

        self.move_timer = self.create_timer(0.1, self.move_police_turtle)

    def spawn_turtle(self):
        if self.spawned:
            return
        
        self.spawn_client = self.create_client(Spawn, 'spawn')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')
        
        request = Spawn.Request()
        request.x, request.y, request.theta = random.uniform(0, 11), random.uniform(0, 11), random.uniform(0, 2 * math.pi)
        request.name = 'police_turtle'
        future = self.spawn_client.call_async(request)
        future.add_done_callback(self.spawn_callback)
        self.spawned = True

    def spawn_callback(self, future):
        try:
            if future.result().name == 'police_turtle':
                self.get_logger().info('Police turtle spawned successfully')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def noisy_pose_callback(self, msg):
        self.rt_noisy_pose = msg

    def police_pose_callback(self, msg):
        self.police_pose = msg

    def move_police_turtle(self):
        if not self.spawned:
            return
        
        dx = self.rt_noisy_pose.x - self.police_pose.x
        dy = self.rt_noisy_pose.y - self.police_pose.y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance <= 1.5:
            self.get_logger().info('Chase is over')
            self.cmd_vel_pub.publish(Twist())
            return
        
        angle_to_target = math.atan2(dy, dx)
        angle_diff = (angle_to_target - self.police_pose.theta + math.pi) % (2 * math.pi) - math.pi
        
        twist_msg = Twist()
        twist_msg.linear.x = self.pt_speed
        twist_msg.angular.z = angle_diff
        self.cmd_vel_pub.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    runner_turtle = RunnerTurtle()
    police_turtle = PoliceTurtle()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(runner_turtle)
    executor.add_node(police_turtle)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        runner_turtle.destroy_node()
        police_turtle.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
