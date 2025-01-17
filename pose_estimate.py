import rclpy
import sys
import argparse
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class SendInitialPose(Node):
    def __init__(self,x,y,z,w):
        super().__init__('send_initial_pose')
        
        # Publisher for setting initial pose
        self.initial_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        # Create and publish initial pose directly with assigned values
        self.set_initial_pose(x,y,z,w)

    def set_initial_pose(self,x,y,z,w):
        # Create a PoseWithCovarianceStamped message
        initial_pose = PoseWithCovarianceStamped()

        # Set the header
        initial_pose.header.frame_id = 'map'  # Correct frame for your system
        initial_pose.header.stamp = self.get_clock().now().to_msg()  # Timestamp

        # Directly assign position and orientation values
        initial_pose.pose.pose.position.x = x
        initial_pose.pose.pose.position.y = y
        initial_pose.pose.pose.position.z = 0.0  # 2D pose, z = 0

        initial_pose.pose.pose.orientation.z = z
        initial_pose.pose.pose.orientation.w = w

        # Publish the initial pose
        self.initial_pose_publisher.publish(initial_pose)
        self.get_logger().info(f'Initial pose set: x=3633.21, y=73649.0, orientation_z=-0.666691, orientation_w=0.745334')

        # Shutdown immediately after publishing
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description="Run the Python script with arguments.")
    parser.add_argument('--initial_pose_x', type=float, help="First argument")
    parser.add_argument('--initial_pose_y', type=float, help="Second argument")
    parser.add_argument('--orientation_z', type=float, help="Third argument")
    parser.add_argument('--orientation_w', type=float, help="Fourth argument")
    args = parser.parse_args()
    node = SendInitialPose(args.initial_pose_x,args.initial_pose_y,args.orientation_z,args.orientation_w)

if __name__ == '__main__':
    main()

