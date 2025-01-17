import rclpy
import sys
import argparse
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from autoware_adapi_v1_msgs.srv import ChangeOperationMode
import rclpy.logging

import time

class SendGoalPose(Node):
    def __init__(self,x,y,z,w,halt):
        super().__init__('send_goal_pose')
        
        

        # Publisher for sending goal pose
        self.goal_publisher = self.create_publisher(PoseStamped, '/planning/mission_planning/goal', 10)
        
        self.auto_mode_client = self.create_client(ChangeOperationMode, '/api/operation_mode/change_to_autonomous')
        
        # Create and publish goal pose directly with assigned values
        self.set_goal_pose(x,y,z,w,halt)

    
    def change_to_auto_mode(self):
        # Create a request object for the service
        request = ChangeOperationMode.Request()

        # Call the service and wait for the result
        future = self.auto_mode_client.call_async(request)
        
        # Handle the response
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Successfully switched to autonomous mode.')
        else:
            self.get_logger().error('Failed to switch to autonomous mode.')

        # Shutdown the node after calling the service
        self.get_logger().info("Shutting down the node.")
        self.destroy_node()
        rclpy.shutdown()
    
    
    def set_goal_pose(self,x,y,z,w,halt):
        # Create a PoseStamped message
        goal_pose = PoseStamped()

        # Set the header
        goal_pose.header.frame_id = 'map'  # Correct frame for your system
        goal_pose.header.stamp = self.get_clock().now().to_msg()  # Timestamp

        # Directly assign position and orientation values for 2D goal
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0  # 2D goal, z = 0

        goal_pose.pose.orientation.z = z
        goal_pose.pose.orientation.w = w

        # Publish the goal pose
        self.goal_publisher.publish(goal_pose)

        # Optionally, you can add an info message for confirmation (though it's suppressed)
        self.get_logger().info(f'')
        
        time.sleep(halt)
        
        self.change_to_auto_mode()

        # Shutdown the node after publishing
        self.get_logger().info("Shutting down the node.")

def main(args=None):
    rclpy.init(args=args)  # Initialize rclpy context before creating the node
    parser = argparse.ArgumentParser(description="Run the Python script with arguments.")
    parser.add_argument('--goal_pose_x', type=float, help="First argument")
    parser.add_argument('--goal_pose_y', type=float, help="Second argument")
    parser.add_argument('--orientation_z', type=float, help="Second argument")
    parser.add_argument('--orientation_w', type=float, help="Third argument")
    parser.add_argument('--halt', type=float, help="Fourth argument")
    args = parser.parse_args()
    node = SendGoalPose(args.goal_pose_x,args.goal_pose_y,args.orientation_z,args.orientation_w,args.halt)  # Create the node
      # Shutdown rclpy after node is destroyed

if __name__ == '__main__':
    main()

