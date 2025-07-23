import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
import math
import random

class RobberTurtle(Node):
    def __init__(self):
        super().__init__('robber_turtle')
        self.speed = 3.2  # RT speed
        self.radius = 5.0  # RT circle radius
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.real_pose_pub = self.create_publisher(Pose, 'rt_real_pose', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.timer = self.create_timer(5.0, self.publish_pose)
        self.move_timer = self.create_timer(0.1, self.move_turtle)

        self.current_pose = Pose()
        self.caught = False
    
    def pose_callback(self, msg):
        self.current_pose = msg
    
    def move_turtle(self):
        if self.caught:
            return
        
        twist_msg = Twist()
        twist_msg.linear.x = self.speed
        twist_msg.angular.z = self.speed / self.radius  # Circular motion equation
        self.cmd_vel_pub.publish(twist_msg)
    
    def publish_pose(self):
        if not self.caught:
            self.real_pose_pub.publish(self.current_pose)

class PoliceTurtle(Node):
    def __init__(self):
        super().__init__('police_turtle')
        self.spawned = False
        self.speed = 1.5  # PT speed
        self.catch_distance = 0.5  # Capture distance
        
        self.spawn_timer = self.create_timer(10.0, self.spawn_turtle)
        self.real_pose_sub = self.create_subscription(Pose, 'rt_real_pose', self.real_pose_callback, 10)
        self.police_pose_sub = self.create_subscription(Pose, '/police_turtle/pose', self.police_pose_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/police_turtle/cmd_vel', 10)
        self.move_timer = self.create_timer(0.1, self.move_police_turtle)
        
        self.rt_pose = Pose()
        self.pt_pose = Pose()
    
    def spawn_turtle(self):
        if self.spawned:
            return

        self.spawn_client = self.create_client(Spawn, 'spawn')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')
        
        x, y, theta = random.uniform(2.0, 8.0), random.uniform(2.0, 8.0), random.uniform(0, 2 * math.pi)
        request = Spawn.Request()
        request.x, request.y, request.theta, request.name = x, y, theta, 'police_turtle'
        
        future = self.spawn_client.call_async(request)
        future.add_done_callback(self.spawn_callback)
        self.spawned = True
    
    def spawn_callback(self, future):
        try:
            response = future.result()
            if response.name == 'police_turtle':
                self.get_logger().info('Police Turtle spawned successfully')
        except Exception as e:
            self.get_logger().error(f'Spawn failed: {e}')
    
    def real_pose_callback(self, msg):
        self.rt_pose = msg
    
    def police_pose_callback(self, msg):
        self.pt_pose = msg
    
    def move_police_turtle(self):
        if not self.spawned:
            return
        
        dx, dy = self.rt_pose.x - self.pt_pose.x, self.rt_pose.y - self.pt_pose.y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance <= self.catch_distance:
            self.get_logger().info('RT caught!')
            return
        
        angle_to_target = math.atan2(dy, dx)
        angle_diff = (angle_to_target - self.pt_pose.theta + math.pi) % (2 * math.pi) - math.pi
        
        twist_msg = Twist()
        twist_msg.linear.x = min(self.speed, distance)  # Adjust speed dynamically
        twist_msg.angular.z = angle_diff * 2.0  # Smooth turning
        
        self.cmd_vel_pub.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    robber_turtle = RobberTurtle()
    police_turtle = PoliceTurtle()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(robber_turtle)
    executor.add_node(police_turtle)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        robber_turtle.destroy_node()
        police_turtle.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

