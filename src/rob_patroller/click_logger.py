import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class ClickLogger(Node):
    def __init__(self):
        super().__init__("click_logger")

        self.declare_parameter("max_points", 5)
        self.max_points = self.get_parameter("max_points").value

        package_path = get_package_share_directory('rob_patroller')
        self.yaml_path = os.path.join(package_path, "config", "clicked_points.yaml")

        self.points = []

        self.subscription = self.create_subscription(
            PointStamped,
            "/clicked_point",
            self.callback,
            10
        )

        self.get_logger().info(f"ClickLogger started. Saving {self.max_points} points.")

    def callback(self, msg):
        if len(self.points) >= self.max_points:
            self.get_logger().info("Already stored max_points.")
            return

        p = [msg.point.x, msg.point.y]
        self.points.append(p)
        self.get_logger().info(f"Stored point: {p}")

        if len(self.points) == self.max_points:
            data = {"points": self.points}
            os.makedirs(os.path.dirname(self.yaml_path), exist_ok=True)
            with open(self.yaml_path, "w") as f:
                yaml.dump(data, f)
            self.get_logger().info(f"Saved to {self.yaml_path}")


def main():
    rclpy.init()
    node = ClickLogger()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
