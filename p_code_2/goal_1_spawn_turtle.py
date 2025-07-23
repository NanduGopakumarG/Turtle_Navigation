import rclpy
import random
from rclpy.node import Node
from turtlesim.srv import Spawn

class TurtleSpawner(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        self.cli = self.create_client(Spawn, "/spawn")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for spawn service...")

        self.req = Spawn.Request()
        self.req.x = random.uniform(1.0, 9.0)
        self.req.y = random.uniform(1.0, 9.0)
        self.req.theta = random.uniform(0.0, 6.28)

        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self.spawn_callback)

    def spawn_callback(self, future):
        response = future.result()
        self.get_logger().info(f"Spawned new turtle: {response.name}")

def main():
    rclpy.init()
    node = TurtleSpawner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
