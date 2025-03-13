#!/usr/bin/env python3

import math
import numpy as np
import time
import roboticstoolbox as rtb
import rclpy
from rclpy.node import Node
from pyquaternion import Quaternion
from roboticstoolbox import DHRobot, RevoluteDH, RevoluteMDH
from spatialmath.base import transl,rpy2tr
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped 
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion as Quat
from scipy.spatial.transform import Rotation as R
from builtin_interfaces.msg import Duration


class IIWAJointPos(Node):
    
    def __init__(self):
        super().__init__('iiwa_joint_pos')

        # Kinematic chain
        self.max_velocity = [1.5, 1.5, 1.5, 2.0, 2.0, 3.0, 3.0]
        self.max_position = [2.9147, 2.0594, 2.9147, 2.0594 ,2.9147, 2.0594, 3.0194]
        tool_xyz = [0.0, 0.0, 0.0]
        self.kuka_robot()
        self.tool = transl(float(tool_xyz[0]), float(tool_xyz[1]), float(tool_xyz[2]))
        self.kuka.tool = self.tool

        # Publisher --> joint position commands 
        self.iiwa_pos_pub = self.create_publisher(JointTrajectory, "/iiwa_arm_controller/joint_trajectory", 1)
        self.timer_period = 0.1  # seconds
        self.time_elapsed = 0.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.main_loop)
        
        # Subscriber --> joint states
        self.q = [0.0, 0.7854, 0.0, -1.3962, 0.0, -0.6109, 0.0] # current joint pos
        self.q_target = self.q # target joint pos
        # joint_states_sub = ... (To do, Q1)

        # Subscriber --> gui commands (To do, Q4)

        # Cartesian variables
        self.pose_current = self.MGD(self.q) # current cart pos
        self.pose_target = self.MGD(self.q) # target cart pos

    def pose_error(self, pose_target, pose_current):
        # To do (Q2 and Q3)
        dX = np.array([
            0.0, 
            0.0, 
            0.0,
            0.0,
            0.0,
            0.0])
        return(dX)

    def kuka_robot(self): # MDH for Kuka IIWA 14
        L = [
            RevoluteMDH(a=0.0,   d=0.36,   alpha=0.0,   qlim=np.array([-self.max_position[0], self.max_position[0]])),
            RevoluteMDH(a=0.0,   d=0.0,    alpha=-np.pi/2,    qlim=np.array([-self.max_position[1], self.max_position[1]])),
            RevoluteMDH(a=0.0,   d=0.42,    alpha=np.pi/2,    qlim=np.array([-self.max_position[2], self.max_position[2]])),
            RevoluteMDH(a=0.0,   d=0.0,    alpha=np.pi/2,   qlim=np.array([-self.max_position[3], self.max_position[3]])),
            RevoluteMDH(a=0.0,   d=0.4,    alpha=-np.pi/2,   qlim=np.array([-self.max_position[4], self.max_position[4]])),
            RevoluteMDH(a=0.0,   d=0.0,    alpha=-np.pi/2,    qlim=np.array([-self.max_position[5], self.max_position[5]])),
            RevoluteMDH(a=0.0,   d=0.3,  alpha=np.pi/2,   qlim=np.array([-self.max_position[6], self.max_position[6]])),
        ]
        self.kuka = DHRobot(L, name="kuka")

    def MGD(self, data_q):
        T = self.kuka.fkine(data_q)
        TT = np.array([[T.n[0], T.o[0], T.a[0], T.t[0]], 
                       [T.n[1], T.o[1], T.a[1], T.t[1]], 
                       [T.n[2], T.o[2], T.a[2], T.t[2]], 
                       [0, 0, 0, 1]])
        quat = Quaternion(matrix = TT)
        FK = PoseStamped()
        FK.pose.position.x = TT[0, 3]
        FK.pose.position.y = TT[1, 3]
        FK.pose.position.z = TT[2, 3]
        FK.pose.orientation.x = quat[1]
        FK.pose.orientation.y = quat[2]
        FK.pose.orientation.z = quat[3]
        FK.pose.orientation.w = quat[0]
        return(FK)
    
    def main_loop(self):

        # --- STARTING CARTESIAN CONTROL --- #
        # 
        # dq = ... To be completed (Q2) 

        # Velocity low-pass filter
        # for i in range(7):
        #     if  abs(dq[i]) > self.max_velocity[i]:
        #         dq[i] = np.sign(dq[i]) * self.max_velocity[i]

        # self.q_target +=  self.timer_period * dq

        # Position limits
        # for i in range(7):
        #     if  abs(self.q_target[i]) > self.max_position[i]:
        #         self.q_target[i] = np.sign(self.q_target[i]) * self.max_position[i]
        
        # --- ENDING CARTESIAN CONTROL --- #
                
        # Create a JointTrajectory message
        joint_trajectory_msg = JointTrajectory()
        joint_trajectory_msg.header = Header()
        for i in range(7):
            joint_trajectory_msg.joint_names.append(f"joint_a{i + 1}")
        joint_trajectory_msg.header.stamp = self.get_clock().now().to_msg()

        # Create a JointTrajectoryPoint
        duration = Duration()
        duration.sec = 0  # Set the seconds part of the duration
        duration.nanosec = 100000000  # Set the nanoseconds part of the duration
        trajectory_point = JointTrajectoryPoint()
        trajectory_point.time_from_start = duration
        trajectory_point.positions = [self.q_target[0], self.q_target[1], self.q_target[2], self.q_target[3], self.q_target[4], self.q_target[5], self.q_target[6]]
        trajectory_point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Add the trajectory point to the JointTrajectory message       
        joint_trajectory_msg.points.append(trajectory_point)

        # Publish the JointTrajectory message
        self.iiwa_pos_pub.publish(joint_trajectory_msg)
        self.get_logger().info('Published JointTrajectory message')
        self.time_elapsed += self.timer_period

def main(args=None):
    rclpy.init(args=args)
    iiwa_joint_pos_node = IIWAJointPos()
    try:
        rclpy.spin(iiwa_joint_pos_node)
    except KeyboardInterrupt:
        pass
    finally:
        iiwa_joint_pos_node.destroy_node()
        rclpy.shutdown()
            
if __name__ == '__main__':
    main()
