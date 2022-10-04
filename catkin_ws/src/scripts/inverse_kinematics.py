#!/usr/bin/env python3
"""
Inverse kinematics
"""

# Always need this
import rospy

import numpy as np

# Import message types
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

def inverse_kinematics(end_eff, link_lengths) -> JointState:
    rospy.loginfo(f'Got desired pose\n[\n\tpos:\n{pose.position}\nrot:\n{pose.orientation}\n]')
    pub.publish(joint_states(end_eff, link_lengths))


def joint_states(end_eff: list) -> JointState:
    """
    Calculates the joint angles of the robot given the position and orientation 
    of the end effector

    Parameters:
        end_eff (array): the end effector position in the form a transformation matrix [x, y, z]
        link_lengths(list): the link lengths of the robot in the form of [l1, l2, l3, l4] TODO: hard code these in

    Returns:
        (list, list): the two options for the robot joint angles

    """
    # Create message of type JointState
    msg = JointState(
        # Set header with current time
        header=Header(stamp=rospy.Time.now()),
        # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
        name=['joint_1', 'joint_2', 'joint_3', 'joint_4']
    )

    # 2 solutions available
    joint_angles_1 = []
    joint_angles_2 = []

    px = end_eff[0]
    py = end_eff[1]
    pz = end_eff[2]

    phi = np.pi / 2 # Can be 0 or 90, use 90 for now

    # mm link lengths (from cad)

    l1 = 55.62 # link_length[0]
    l2 = 117.5 #link_lengths[1]
    l3 = 95 #link_lengths[2]
    l4 = 98.19 #link_lengths[3]

    c_theta_3 = (px ** 2 + py ** 2 - l2 ** 2 - l3 ** 2) / (2 * l2 * l3)

    # First solution
    theta_1a = np.arctan2(py, px)
    theta_3a = np.arctan2(c_theta_3, np.sqrt(1 - c_theta_3 ** 2))
    theta_2a = np.arctan2(px, pz) - np.arctan2(l2 + l3 * np.cos(theta_3a), l3 * np.sin(theta_3a))
    theta_4a = phi - (theta_2a + theta_3a)

    # TODO: Check if theta 4 is valid, change phi if not


    # Second solution
    theta_1b = np.arctan2(py, px)
    theta_3b = np.arctan2(c_theta_3, - np.sqrt(1 - c_theta_3 ** 2))
    theta_2b = np.arctan2(px, pz) - np.arctan2(l2 + l3 * np.cos(theta_3b), l3 * np.sin(theta_3b))
    theta_4b = phi - (theta_2b + theta_3b)

    # TODO: Check if theta 4 is valid, change phi if not

    # TODO: LOGIC TO CHOOSE JOINT ANGLES
    msg.position = [
        theta_1a,
        theta_2a,
        theta_3a,
        theta_4a
    ]

    return msg

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