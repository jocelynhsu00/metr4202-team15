#!/usr/bin/env python3
"""
Inverse kinematics
"""

# Always need this
import rospy
import math

import numpy as np

# Import message types
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

def inverse_kinematics(pose: Pose) -> JointState:
    global pub 

    # INVERSE KINEMATICS

    # Create message of type JointState
    msg = JointState(
        # Set header with current time
        header=Header(stamp=rospy.Time.now()),
        # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
        name=['joint_1', 'joint_2', 'joint_3', 'joint_4']
    )
    
    # mm link lengths (from cad)
    l1 = 75 
    l2 = 117.5 
    l3 = 95 
    l4 = 85 

    # End eff posiiton
    x_end = pose.position.x 
    x_end_abs = abs(pose.position.x)
    y_end = pose.position.y- l1
    z_end = pose.position.z 

    # mm link lengths (from cad)
    l1 = 75 # link_length[0] (used for horizontal grab)
    l2 = 117.5 #link_lengths[1]
    l3 = 95 #link_lengths[2]
    l4 = 85 #link_lengths[3]

    # Check reachable within workspace
    # Gripper at 90 deg
    max_length_gripper_90 = l2 + l3

    # Gripper at 0 deg
    max_length_gripper_0 = l2 + l4 + np.sqrt(l3**2 - l1**2)

    gripper_90_valid = False

    gripper_0_valid = False

    if x_end_abs < max_length_gripper_0:
        gripper_0_valid = True

    if x_end_abs < max_length_gripper_90:
        gripper_90_valid = True

    # Try gripper 90 deg sln first
    if gripper_90_valid:
        psi = np.pi/2

        # Find pos at dynamixel 4
        x_4 = x_end_abs
        y_4 = y_end + l4
        z_4 = z_end

        # Elbow up
        c_theta_3 = (x_4**2+y_4**2-l2**2-l3**2)/(2*l2*l3)
        theta_3 = np.arctan2(-np.sqrt(1-c_theta_3), c_theta_3)
        theta_2 = np.arctan2(y_4, x_4) - np.arctan2(l3*np.sin(theta_3), l2 + l3*np.cos(theta_3))
        theta_4 = -psi - theta_2 - theta_3
        theta_1 = np.arctan(z_4/x_4)

        # Convert to dynamixel input TODO: check if any of the dynamixel values need to be negated/flipped (so far seems like 4 needs to be)
        theta_3 = -theta_3

        theta_2 = np.pi/2 - theta_2
        if x_4 < 0:
            theta_2 = - theta_2
        
        # Negating
        theta_4 = -theta_4
        theta_3 = -theta_3
        theta_2 = -theta_2
        theta_1 = -theta_1

        # Negate if -ve x
        if x_end < 0:
            # Negating
            theta_4 = -theta_4
            theta_3 = -theta_3
            theta_2 = -theta_2


        theta_list  = [theta_1, theta_2, theta_3, theta_4]
        # Check limits
        # TODO: COLLISION AVOIDANCE
        all_valid = True
        for index, theta in enumerate(theta_list):
            # Check dynamixel limits (want to avoid going over -90, 90 deg as it damages the wiring)
            if index != 3:
                if theta < -np.deg2rad(140) or theta > np.deg2rad(140) or math.isnan(theta):
                    all_valid = False
                    break
            else:
                if theta < -np.deg2rad(110) or theta > np.deg2rad(110) or math.isnan(theta):
                    all_valid = False
                    break
        
        if all_valid:
            msg.position = theta_list

        elif not all_valid and gripper_0_valid:
            # Try gripper 0 position
            psi = 0

            # Find pos at dynamixel 4
            x_4 = x_end_abs - l4
            y_4 = y_end
            z_4 = z_end

            # Elbow up
            c_theta_3 = (x_4**2+y_4**2-l2**2-l3**2)/(2*l2*l3)
            # IF x is negative, elbow up is +ve sln rather than negative
            if x_4 < 0:
                theta_3 = np.arctan2(np.sqrt(1-c_theta_3), c_theta_3)
            elif x_4 > 0:
                theta_3 = np.arctan2(-np.sqrt(1-c_theta_3), c_theta_3)
            theta_2 = np.arctan2(y_4, x_4) - np.arctan2(l3*np.sin(theta_3), l2 + l3*np.cos(theta_3))
            theta_4 = -psi - theta_2 - theta_3
            theta_1 = np.arctan(z_4/x_4)

            # Convert to dynamixel input TODO: check if any of the dynamixel values need to be negated/flipped (so far seems like 4 needs to be)
            theta_3 = -theta_3
            theta_2 = np.pi/2 - theta_2
            if x_4 < 0:
                theta_2 = - theta_2

            # Negating
            theta_4 = -theta_4
            theta_3 = -theta_3
            theta_2 = -theta_2
            theta_1 = -theta_1

            # Negate if -ve x
            if x_end < 0:
                # Negating
                theta_4 = -theta_4
                theta_3 = -theta_3
                theta_2 = -theta_2

            theta_list  = [theta_1, theta_2, theta_3, theta_4]
            # Check limits
            # TODO: COLLISION AVOIDANCE
            all_valid = True
            for index, theta in enumerate(theta_list):
                # Check dynamixel limits (want to avoid going over -90, 90 deg as it damages the wiring)
                if index != 3:
                    if theta < -np.deg2rad(140) or theta > np.deg2rad(140) or math.isnan(theta):
                        all_valid = False
                        # This means both sln are invalid: return to zero config
                        msg.position = [0, 0, 0, 0]
                        print('No valid sln, return to zero config')
                        break
                else:
                    if theta < -np.deg2rad(110) or theta > np.deg2rad(110) or math.isnan(theta):
                        all_valid = False
                        # This means both sln are invalid: return to zero config
                        msg.position = [0, 0, 0, 0]
                        print('No valid sln, return to zero config')
                        break
            
            if all_valid:
                msg.position = theta_list

        deg = []
        for theta in theta_list:
            deg.append(np.rad2deg(theta))
        print(deg)
        rospy.loginfo(f'Got desired pose\n[\n\tpos:\n{pose.position}\nrot:\n{pose.orientation}\n]')
        pub.publish(msg)


def main():
    global pub
    # Create publisher
    pub = rospy.Publisher(
        'desired_joint_states', # Topic name
        JointState, # Message type
        queue_size=10 # Topic size (optional)
    )

    # Create subscriber
    sub = rospy.Subscriber(
        'desired_pose', # Topic name
        Pose, # Message type
        inverse_kinematics # Callback function (required)
    )

    # Initialise node with any node name
    rospy.init_node('joint_states')

    # You spin me right round baby, right round...
    # Just stops Python from exiting and executes callbacks
    rospy.spin()

if __name__ == '__main__':
    main()