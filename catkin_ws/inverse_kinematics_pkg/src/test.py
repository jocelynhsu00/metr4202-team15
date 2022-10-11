from math import atan2
from numpy import sin, cos, sqrt
from numpy import pi, abs

def inverse_kinematics(
        end_effector_pos : list,
        link_length : list,
        pitch_angle : int
    ):
    """Calculate the joint positions from an end effector configuration.
    
    Args:
        pose: The end effector configuration.
        lengths: The lengths of each links.

    Returns:
        The angles of each joint to the end effector.
    """


    #To test it:
    #cd ~/catkin_ws
    #catkin_make
    #source devel/setup.bash
    #roslaunch dynamixel_interface dynamixel_interface_controller.launch
    #rostopic pub /desired_joint_states /msg_JointState [TAB]
    #echo 0 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb

    # Define variables (Don't exceed robot arm length)
    # Coordinates of end-effector (cubes)
    x = end_effector_pos[0] # y-coordinate of end-effector
    y = end_effector_pos[1] # x-coordinate of end-effector
    z = end_effector_pos[2] # z-coordinate of end-effector
    #*****^^^This will be replaced by subscriber/publisher***********# 

    alpha = pitch_angle # angle of gripper (0 to 90), set to 90 [radians]

    # Dimentsions of the robot (in mm)
    L1 = link_length[0]
    L2 = link_length[1]
    L3 = link_length[2]
    L4 = link_length[3]
    # Position of joint 3
    pxy = sqrt(x**2 + y**2) - L4*cos(alpha) 
    pz = z - L4*sin(alpha) - L1 #joint 3 y coordinate
    
    C_theta_3 = (pxy**2 + pz**2 - L2**2 - L3**2) / (2 * L2 * L3) 

    #Angle Calculations (in radians)
    theta_1 = atan2(x,y)
    theta_3 = atan2(-sqrt(abs(1-C_theta_3**2)), C_theta_3)
    theta_2 = (atan2(pz,pxy)  -  atan2(L3*sin(theta_3),  L2+L3*cos(theta_3)))
    theta_4 = (alpha - theta_2 - theta_3) % (2*pi)

    if theta_4 > pi:
        theta_4 = -((2*pi) - theta_4)

    #Update angles
    theta_1 = theta_1
    theta_2 = pi/2-theta_2
    theta_3 = -theta_3
    theta_4 = theta_4
    # if theta_4 > pi:
    #     theta_4 = -((2*pi) - theta_4)

    # Publish thetas to the robot
    # return list of thetas
    return [theta_1, theta_2, theta_3, theta_4]

