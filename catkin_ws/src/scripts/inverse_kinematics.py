import numpy as np

def inv_kin(end_eff: list, link_lengths:list):
    """
    Calculates the joint angles of the robot given the position and orientation 
    of the end effector

    Parameters:
        end_eff (array): the end effector position in the form a transformation matrix [x, y, z]
        link_lengths(list): the link lengths of the robot in the form of [l1, l2, l3, l4]

    Returns:
        (list, list): the two options for the robot joint angles

    """
    # 2 solutions available
    joint_angles_1 = []
    joint_angles_2 = []

    px = end_eff[0]
    py = end_eff[1]
    pz = end_eff[2]

    phi = np.pi / 2 # Can be 0 or 90

    l1 = link_lengths[0]
    l2 = link_lengths[1]
    l3 = link_lengths[2]
    l4 = link_lengths[3]

    c_theta_3 = (px ** 2 + py ** 2 - l2 ** 2 - l3 ** 2) / (2 * l2 * l3)

    # First solution
    theta_1a = np.arctan2(py, px)
    theta_3a = np.arctan2(c_theta_3, np.sqrt(1 - c_theta_3 ** 2))
    theta_2a = np.arctan2(px, pz) - np.arctan2(l2 + l3 * np.cos(theta_3a), l3 * np.sin(theta_3a))
    theta_4a = phi - (theta_2a + theta_3a)

    # Check if theta 4 is valid, change phi if not

    joint_angles_1 = [theta_1a, theta_2a, theta_3a, theta_4a]

    # Second solution
    theta_1b = np.arctan2(py, px)
    theta_3b = np.arctan2(c_theta_3, - np.sqrt(1 - c_theta_3 ** 2))
    theta_2b = np.arctan2(px, pz) - np.arctan2(l2 + l3 * np.cos(theta_3b), l3 * np.sin(theta_3b))
    theta_4b = phi - (theta_2b + theta_3b)

    # Check if theta 4 is valid, change phi if not

    joint_angles_2 = [theta_1b, theta_2b, theta_3b, theta_4b] 

    return joint_angles_1, joint_angles_2
