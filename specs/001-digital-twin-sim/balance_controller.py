#!/usr/bin/env python3

"""
Simple balance controller for the humanoid robot
This controller keeps the humanoid standing upright by adjusting leg positions
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class BalanceController(Node):
    def __init__(self):
        super().__init__('balance_controller')
        
        # Publisher for joint trajectory commands
        self.joint_cmd_pub = self.create_publisher(
            JointTrajectory, 
            '/simple_humanoid_position_controller/command', 
            10
        )
        
        # Subscriber for joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.balance_control_loop)  # 10Hz
        
        # Joint names for the legs
        self.joint_names = [
            'torso_to_left_upper_leg',
            'left_upper_leg_to_lower_leg',
            'torso_to_right_upper_leg',
            'right_upper_leg_to_lower_leg'
        ]
        
        self.joint_positions = [0.0] * len(self.joint_names)
        self.standing_pos = [-0.1, 0.2, -0.1, 0.2]  # Default standing position
        
        self.get_logger().info('Balance controller initialized')

    def joint_state_callback(self, msg):
        """Callback to update current joint positions"""
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.joint_positions[i] = msg.position[idx]

    def balance_control_loop(self):
        """Main control loop to maintain balance"""
        # Create a joint trajectory message
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names
        
        # Create a trajectory point
        point = JointTrajectoryPoint()
        
        # For simple balance, maintain standing position
        # In a real implementation, this would adjust based on IMU feedback
        point.positions = self.standing_pos
        
        # Set velocities to 0 to move smoothly
        point.velocities = [0.0] * len(self.joint_names)
        
        # Set accelerations to 0
        point.accelerations = [0.0] * len(self.joint_names)
        
        # Add the point to the trajectory with a time from start
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 50000000  # 0.05 seconds
        
        traj_msg.points.append(point)
        
        # Publish the trajectory
        self.joint_cmd_pub.publish(traj_msg)

def main(args=None):
    rclpy.init(args=args)
    
    balance_controller = BalanceController()
    
    try:
        rclpy.spin(balance_controller)
    except KeyboardInterrupt:
        pass
    finally:
        balance_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()