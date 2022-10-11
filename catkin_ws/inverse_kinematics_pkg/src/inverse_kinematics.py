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
    
    # msg.position = [
    #     -np.deg2rad(30),
    #     np.deg2rad(30),
    #     np.deg2rad(30),
    #     -np.deg2rad(30)
    # ]
    # rospy.loginfo('Test 30 deg')
    # pub.publish(msg)
    # End eff posiiton
    px_end = pose.position.x #end_eff[0]
    py_end = pose.position.y #end_eff[1]
    pz_end = pose.position.z #nd_eff[2]

    print(px_end, py_end, pz_end)

    # Ensure no division by 0
    if px_end != 0 and py_end != 0:
        print('non-zero end eff')

        psi = - np.pi / 2 # Gripper always angled at - 90 deg relative to ground, can be changed if needed

        # mm link lengths (from cad)
        l1 = 53 # link_length[0] (not used)
        l2 = 117.5 #link_lengths[1]
        l3 = 95 #link_lengths[2]
        l4 = 85 #link_lengths[3]

        # Get position of arm at theta 4
        px_4 = px_end - l4 * np.cos(psi)
        py_4 = py_end 
        pz_4 = pz_end - l4 * np.sin(psi)

        # Consider 2dof arm
        theta_3 = np.arccos((px_4**2 + pz_4**2 - (l2**2 + l3**2))/(2*l2*l3))
        theta_2 = np.arctan(pz_4/px_4) - np.arctan((l3*np.sin(theta_3))/(l2+l3*np.cos(theta_3)))

        # Update angles for elbow up solution
        # theta_3 = - theta_3
        theta_2 = np.pi/2 - theta_2

        theta_4 = psi - (theta_2 + theta_3)
        # theta_4 = - theta_4

        theta_1 = np.arctan(py_end/px_end)
        theta_1 = - theta_1

        # Check limits
        theta_list = [theta_1, theta_2, theta_3, theta_4]
        print(theta_list)

        # Assume all valid
        all_valid = True

        for theta in theta_list:
            # Check for invalid
            if theta <= -1.5 or theta >= 1.5:
                # Invalid
                all_valid = False
                break

        # Only publish if angles are valid
        if all_valid:
            msg.position = theta_list
            print("VALID:", theta_list)
            rospy.loginfo(f'Got desired pose\n[\n\tpos:\n{pose.position}\nrot:\n{pose.orientation}\n]')
            pub.publish(msg)
        else:
            msg.position = [0, 0, 0, 0]
            rospy.loginfo('use 0 config')
            pub.publish(msg)

    else:
        msg.position = [0, 0, 0, 0]
        rospy.loginfo('use 0 config')
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