# Pseudocode for follower behavior
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class Follower(Node):
    def _init_(self):
        super()._init_('follower_node')
        self.leader_sub = self.create_subscription(PoseStamped, '/uav1/pose', self.follow_leader, 10)
        self.local_pub = self.create_publisher(PoseStamped, '/uav2/setpoint_position/local', 10)

    def follow_leader(self, msg):
        new_pose = PoseStamped()
        new_pose.header = msg.header
        new_pose.pose.position.x = msg.pose.position.x - 2.0
        new_pose.pose.position.y = msg.pose.position.y
        new_pose.pose.position.z = msg.pose.position.z
        self.local_pub.publish(new_pose)

rclpy.init()
node = Follower()
rclpy.spin(node)