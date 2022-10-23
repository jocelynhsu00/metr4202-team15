#!/usr/bin/env python3
"""
Inverse kinematics
"""

# Always need this
import rospy
import math
import time

import numpy as np

# Import message types
from std_msgs.msg import Header, String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

def inverse_kinematics(pose: Pose) -> JointState:
    # Setup
    global pub_pose 
    pub_gripper_pos
    success = 0
    gripper_pos = -1
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

    total_height = l1 + l2 + l3 + l4

    # End eff posiiton
    x_end = pose.position.x 
    x_end_abs = abs(pose.position.x)
    y_end = pose.position.y- l1
    z_end = pose.position.z 
    print(x_end_abs, y_end, z_end)

    if z_end < 0:
        z_end -= 10

    dist_from_centre = np.sqrt(x_end **2 + z_end **2)

    print(y_end)


    # Check reachable within workspace
    # Gripper at 90 deg
    max_length_gripper_90 = l2 + l3

    # Gripper at 0 deg
    max_length_gripper_0 = l2 + l4 + np.sqrt(l3**2 - l1**2)

    print("Max lengths", max_length_gripper_90, max_length_gripper_0)

    gripper_90_valid = False

    gripper_0_valid = False

    if dist_from_centre < max_length_gripper_0:
        gripper_0_valid = True

    if dist_from_centre < max_length_gripper_90:
        gripper_90_valid = True

    print("Check max:", gripper_90_valid, gripper_0_valid)

    all_valid = None

    theta_list = None
     # Try gripper 90 deg sln first
    if gripper_90_valid:
        if (x_end > 80 or x_end < -40) and (y_end > -40 or y_end > total_height):
            print("Passed floor collision and env collision check")
            psi = np.pi/2

            # Find pos at dynamixel 4
            x_4 = x_end_abs + 25
            y_4 = y_end + l4
            z_4 = z_end

            dist_from_centre_4 = np.sqrt(x_4**2 + z_4**2)


            # Elbow up
            c_theta_3 = (dist_from_centre_4**2+y_4**2-l2**2-l3**2)/(2*l2*l3)
            theta_3 = np.arctan2(-np.sqrt(1-c_theta_3), c_theta_3)
            theta_2 = np.arctan2(y_4, dist_from_centre_4) - np.arctan2(l3*np.sin(theta_3), l2 + l3*np.cos(theta_3))
            theta_4 = -psi - theta_2 - theta_3
            theta_1 = np.arctan(z_4/x_4)

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
                theta_1 = -theta_1


            theta_list  = [theta_1, theta_2, theta_3, theta_4]
            # print("90 deg test", np.rad2deg(theta_list))
            # Check limits
            all_valid = True
            for index, theta in enumerate(theta_list):
                print("90 deg theta:", index, np.rad2deg(theta))
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
                gripper_pos = 90

    # Try gripper 0 pos
    if (all_valid is None or all_valid is False) and gripper_0_valid:
        if (x_end > 80 or x_end < -40) and (y_end > -25 or y_end > total_height):
            print("Passed floor collision and env collision check")
            print("Trying 0")
            # Try gripper 0 position
            psi = 0

            # Find pos at dynamixel 4
            # x_4 = x_end_abs - l4 - 26
            y_4 = y_end
            # z_4 = z_end

            theta = np.arctan(z_end/x_end_abs)
            x_4 = x_end_abs - l4 * np.cos(theta) - 22*np.cos(theta)
            z_4 = z_end - l4 * np.sin(theta) - 22*np.sin(theta)

            dist_from_centre_4 = np.sqrt(x_4**2 + z_4**2)

            # Elbow up
            c_theta_3 = (dist_from_centre_4**2+y_4**2-l2**2-l3**2)/(2*l2*l3)
            # IF x is negative, elbow up is +ve sln rather than negative
            if x_4 < 0:
                theta_3 = np.arctan2(np.sqrt(1-c_theta_3), c_theta_3)
            elif x_4 > 0:
                theta_3 = np.arctan2(-np.sqrt(1-c_theta_3), c_theta_3)
            theta_2 = np.arctan2(y_4, dist_from_centre_4) - np.arctan2(l3*np.sin(theta_3), l2 + l3*np.cos(theta_3))
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
                theta_1 = -theta_1

            theta_list  = [theta_1, theta_2, theta_3, theta_4]
            # Check limits
            all_valid = True
            for index, theta in enumerate(theta_list):
                # Check dynamixel limits (want to avoid going over -90, 90 deg as it damages the wiring)
                if index != 3:
                    if theta < -np.deg2rad(140) or theta > np.deg2rad(140) or math.isnan(theta):
                        all_valid = False
                        # This means both sln are invalid: return to zero config
                        print('No valid sln, move to neutral pos')
                        break
                else:
                    if theta < -np.deg2rad(110) or theta > np.deg2rad(110) or math.isnan(theta):
                        all_valid = False
                        # This means both sln are invalid: return to zero config
                        print('No valid sln, mvoe to neutral pos')
                        break
            
            if all_valid:
                msg.position = theta_list
                gripper_pos = 0

    if theta_list is None:
        theta_list = [0, np.deg2rad(40), np.deg2rad(-66), np.deg2rad(10)]
        msg.position = theta_list
        success = 1
    deg = []
    for theta in theta_list:
        deg.append(np.rad2deg(theta))
    print(deg)
    rospy.loginfo(f'Got desired pose\n[\n\tpos:\n{pose.position}\nrot:\n{pose.orientation}\n]')
    msg.velocity = 2.0
    pub_pose.publish(msg)
    pub_gripper_pos.publish(str(gripper_pos))

    return success


def main():
    global pub_pose
    global pub_gripper_pos
    # Create publisher
    pub_pose = rospy.Publisher(
        'desired_joint_states', # Topic name
        JointState, # Message type
        queue_size=10 # Topic size (optional)
    )

    pub_gripper_pos = rospy.Publisher(
        'gripper_pos', # Topic name
        String, # Message type
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