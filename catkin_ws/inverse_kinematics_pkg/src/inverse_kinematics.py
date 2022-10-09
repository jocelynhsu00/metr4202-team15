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
    
    # End eff posiiton
    px = pose.position.x #end_eff[0]
    py = pose.position.y #end_eff[1]
    pz = pose.position.z #nd_eff[2]

    phi = np.pi / 2 # Can be 0 or 90, use 90 for now

    # mm link lengths (from cad)

    l1 = 53.390 # link_length[0]
    l2 = 117.5 #link_lengths[1]
    l3 = 95 #link_lengths[2]
    l4 = 85 #link_lengths[3]

    c_theta_3 = (px ** 2 + py ** 2 - l2 ** 2 - l3 ** 2) / (2 * l2 * l3)

    # First solution
    theta_1 = np.arctan2(py, px)
    theta_3 = np.arctan2(c_theta_3, np.sqrt(1 - c_theta_3 ** 2))
    theta_2 = np.arctan2(px, pz) - np.arctan2(l2 + l3 * np.cos(theta_3), l3 * np.sin(theta_3))
    theta_4 = phi - (theta_2 + theta_3)

    # Collision detection (ensure all theta values between -pi/2 and pi/2)
    for theta in [theta_1, theta_2, theta_3]:
        # If any values result in collision, use second solution
        if not (-1.5 < theta < 1.5):
            theta_1 = np.arctan2(py, px)
            theta_3 = np.arctan2(c_theta_3, - np.sqrt(1 - c_theta_3 ** 2))
            theta_2 = np.arctan2(px, pz) - np.arctan2(l2 + l3 * np.cos(theta_3), l3 * np.sin(theta_3))
            theta_4 = phi - (theta_2 + theta_3)
            break

   # Check if theta 4 is valid, change phi if not (alternates between 0 and 90, can set to arbitrary value later)
    if not (-1.5 < theta_4 < 1.5):
        phi = 0
        theta_4 = phi - (theta_2 + theta_3)

 
    msg.position = [
        theta_1,
        theta_2,
        theta_3,
        theta_4
    ]

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