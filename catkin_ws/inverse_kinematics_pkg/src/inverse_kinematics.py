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

    # INVERSE KINEMATIC

    # Create flag for if a valid solution exists
    valid_sol = False

    used_second_sol = False

    # Create message of type JointState
    msg = JointState(
        # Set header with current time
        header=Header(stamp=rospy.Time.now()),
        # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
        name=['joint_1', 'joint_2', 'joint_3', 'joint_4']
    )
    
    # End eff posiiton
    px_end = pose.position.x #end_eff[0]
    py_end = pose.position.y #end_eff[1]
    pz_end = pose.position.z #nd_eff[2]

    alpha = np.pi / 2 # Gripper always angled at 90 deg to ground, can be changed if needed

    # mm link lengths (from cad)
    l1 = 54 # link_length[0]
    l2 = 117.5 #link_lengths[1]
    l3 = 95 #link_lengths[2]
    l4 = 85 #link_lengths[3]

    # First calculate position at end of link 3
    px_3 = px_end - l4 * np.cos(alpha)
    py_3 = py_end - l4 * np.sin(alpha)
    pz_3 = pz_end # z pos doesnt change

    c_theta_3 = (px_3 ** 2 + py_3 ** 2 - l2 ** 2 - l3 ** 2) / (2 * l2 * l3)

    # Want elbow up solution
    theta_1 = np.arctan2(px_end, pz_end) # Maybe needs to 90 - theta_1 
    theta_3 = np.arctan2(c_theta_3, - np.sqrt(1 - c_theta_3 ** 2))
    theta_2 = np.arctan2(py_3, px_3) - np.arctan2(l2 + l3 * np.cos(theta_3), l3 * np.sin(theta_3))
    theta_4 = alpha - (theta_2 + theta_3)

    # Convert angles for input to dynamixels
    theta_2 = (np.pi/2) - theta_2

    theta_list = [theta_1, theta_2, theta_3, theta_4]

    # Check limits

    all_valid = True

    for theta in theta_list:
        # Check collision
        if theta < -np.deg2rad(150) or theta > np.deg2rad(150):
            all_valid = False
            break

    if all_valid:
        msg.position = [
            theta_1,
            theta_2,
            theta_3,
            theta_4
        ]
        rospy.loginfo(f'Got desired pose\n[\n\tpos:\n{pose.position}\nrot:\n{pose.orientation}\n]')
        pub.publish(msg)

    else:
        # Set to random pos
        print("Setting to standard pos")
        msg.position = [
            0,
            0,
            1.5,
            0
        ]
        rospy.loginfo('Invalid pose, setting to random pos')
        pub.publish(msg)

    # # Check if angles have been updated 
    # angles_updated = False

    # for theta in theta_list:
    #     # Check collision
    #     if theta < -np.deg2rad(150) or theta > np.deg2rad(150):
    #         # Indicate the angles are being updated
    #         angles_updated = True

    #         # Try orienting gripper at 0 deg to table
    #         alpha = 0
    #         # First calculate position at end of link 3
    #         px_3 = px_end - l4 * np.cos(alpha)
    #         py_3 = py_end - l4 * np.sin(alpha)
    #         pz_3 = pz_end # z pos doesnt change

    #         c_theta_3 = (px_3 ** 2 + py_3 ** 2 - l2 ** 2 - l3 ** 2) / (2 * l2 * l3)

    #         theta_1 = np.arctan2(px_end, pz_end) # Maybe needs to 90 - theta_1 
    #         theta_3 = np.arctan2(c_theta_3, - np.sqrt(1 - c_theta_3 ** 2))
    #         theta_2 = np.arctan2(py_3, px_3) - np.arctan2(l2 + l3 * np.cos(theta_3), l3 * np.sin(theta_3))
    #         theta_4 = alpha - (theta_2 + theta_3)

    #         # Update theta list
    #         theta_list = theta_list = [theta_1, theta_2, theta_3, theta_4]
    #         break

    # # Check updated solution
    # if angles_updated:
    #     all_valid = True
    #     # Check collision
    #     for theta in theta_list:
    #         if theta < -np.deg2rad(150) or theta > np.deg2rad(150):
    #             # Updated solution is invalid
    #             all_valid = False
    #             break

    # # Only publish valid solution
    # if all_valid:
    #     msg.position = [
    #         theta_1,
    #         theta_2,
    #         theta_3,
    #         theta_4
    #     ]
    #     rospy.loginfo(f'Got desired pose\n[\n\tpos:\n{pose.position}\nrot:\n{pose.orientation}\n]')
    #     pub.publish(msg)


#     # Check if theta 4 is valid, change phi if not (alternates between 0 and 90, can set to arbitrary value later)
#     if theta_4 < -1.5 or theta_4 > 1.5:
#         phi = 0
#         theta_4 = phi - (theta_2 + theta_3)

#     # Collision detection (ensure all theta values between -pi/2 and pi/2)
#     for theta in [theta_1, theta_2, theta_3, theta_4]:
#         # If any values result in collision, use second solution
#         if theta < -1.5 or theta > 1.5:
#             theta_1 = np.arctan2(py, px)
#             theta_3 = np.arctan2(c_theta_3, - np.sqrt(1 - c_theta_3 ** 2))
#             theta_2 = np.arctan2(px, pz) - np.arctan2(l2 + l3 * np.cos(theta_3), l3 * np.sin(theta_3))
#             theta_4 = phi - (theta_2 + theta_3)
#             used_second_sol = True
#             break
    
#    # Check if theta 4 is valid, change phi if not (alternates between 0 and 90, can set to arbitrary value later)
#     if not (-1.5 < theta_4 < 1.5):
#         phi = 0
#         theta_4 = phi - (theta_2 + theta_3)

    # msg.position = [
    #     theta_1,
    #     theta_2,
    #     theta_3,
    #     theta_4
    # ]

    # rospy.loginfo(f'Got desired pose\n[\n\tpos:\n{pose.position}\nrot:\n{pose.orientation}\n]')
    # pub.publish(msg)


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
