import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self.client = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        self.client.wait_for_server()

    def send_waypoints(self, waypoints):
        goal = FollowWaypoints.Goal()
        goal.poses = waypoints
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

def main():
    rclpy.init()
    node = WaypointFollower()

    # Define waypoints
    waypoints = []
    for x, y in [(1.0, 1.0), (2.0, 2.0), (3.0, 1.5)]:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = node.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        waypoints.append(pose)

    node.send_waypoints(waypoints)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
