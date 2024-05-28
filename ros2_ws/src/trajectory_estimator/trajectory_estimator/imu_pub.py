import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import pandas as pd
import os
import yaml

script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(script_dir, "../../../../../../.."))
config_path = os.path.join(project_root, "config.yaml")
with open(config_path, "r") as file:
    config = yaml.safe_load(file)

root = config["root"]
ros2_path_imu = root + config["ros2_paths"]["imu"]

class ImuTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__("imu_path")
        self.publisher = self.create_publisher(Path, "imu_path", 10)
        self.timer = self.create_timer(1.0, self.publish_trajectory)
        self.path = Path()
        dataframe = pd.read_csv(ros2_path_imu)
        for cord in dataframe.itertuples():
            pose = PoseStamped()
            pose.header.frame_id = "world"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = cord.x
            pose.pose.position.y = cord.y
            pose.pose.position.z = 0.0
            self.path.poses.append(pose)

    def publish_trajectory(self):
        self.path.header.frame_id = "world"
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.path)

def main(args=None):
    rclpy.init(args=args)
    trajectory_publisher = ImuTrajectoryPublisher()
    rclpy.spin(trajectory_publisher)
    trajectory_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
