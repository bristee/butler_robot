import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
import yaml
from std_msgs.msg import String
import time 

class ButlerNav(Node):
    def __init__(self):
        super().__init__('butler_nav')
        self.client = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        self.client.wait_for_server()
        self.subscription = self.create_subscription(String, 'order_topic', self.process_order, 10)
        self.load_waypoints()

    def load_waypoints(self):
        with open('/home/adads/ros2_ws/src/butler_bot/config/waypoints.yaml', 'r') as file:
            self.waypoints = yaml.safe_load(file)['waypoints']

    def process_order(self, msg):
        table = msg.data
        waypoints = [self.create_pose('kitchen'), self.create_pose(table), self.create_pose('home')]
        self.send_waypoints(waypoints)

    def create_pose(self, location):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x, pose.pose.position.y = self.waypoints[location]
        pose.pose.orientation.w = 1.0
        return pose


#import time  # Import time module

    def send_waypoints(self, waypoints):
        goal = FollowWaypoints.Goal()
        goal.poses = [pose for pose in waypoints if pose]  # Ensure valid poses

        self.get_logger().info(f"Navigating to: {[(p.pose.position.x, p.pose.position.y) for p in goal.poses]}")

        send_goal_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)  # Wait for goal to complete

        self.get_logger().info("Pausing before moving to next waypoint...")
        time.sleep(3)  # Add 3-second delay between waypoints

def main():
    rclpy.init()
    node = ButlerNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
