"""
Inverse kinematics
"""

import numpy as np
import math

def inverse_kinematics(x, y, z):
    # mm link lengths (from cad)
    l1 = 75 # link_length[0] (used for horizontal grab)
    l2 = 117.5 #link_lengths[1]
    l3 = 95 #link_lengths[2]
    l4 = 85 #link_lengths[3]

    # End eff posiiton
    x_end = x
    x_end_abs = abs(x)
    y_end = y - l1# TODO: This does some weird stuff
    z_end = z

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
        # IF x is negative, elbow up is +ve sln rather than negative
        if x_4 < 0:
            theta_3 = np.arctan2(np.sqrt(1-c_theta_3), c_theta_3)
        elif x_4 > 0:
            theta_3 = np.arctan2(-np.sqrt(1-c_theta_3), c_theta_3)

        theta_2 = np.arctan2(y_4, x_4) - np.arctan2(l3*np.sin(theta_3), l2 + l3*np.cos(theta_3))
        theta_4 = -psi - theta_2 - theta_3
        theta_1 = np.arctan(z_4/ x_4)

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

        if x_end < 0:
            # Negating
            theta_4 = -theta_4
            theta_3 = -theta_3
            theta_2 = -theta_2


        theta_list  = [theta_1, theta_2, theta_3, theta_4]
        deg = []
        for theta in theta_list:
            deg.append(np.rad2deg(theta))
        print("90 deg", deg)
        # print(theta_list)
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
            degs = []
            for theta in theta_list:
                degs.append(np.rad2deg(theta))
            return degs

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
            theta_1 = np.arctan(z_4/ x_4)

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

            if x_end < 0:
                # Negating
                theta_4 = -theta_4
                theta_3 = -theta_3
                theta_2 = -theta_2

            theta_list  = [theta_1, theta_2, theta_3, theta_4]
            # print(theta_list)
            # Check limits
            # TODO: COLLISION AVOIDANCE
            all_valid = True
            for index, theta in enumerate(theta_list):
                # Check dynamixel limits (want to avoid going over -90, 90 deg as it damages the wiring)
                if index != 3:
                    if theta < -np.deg2rad(140) or theta > np.deg2rad(140) or math.isnan(theta):
                        all_valid = False
                        # This means both sln are invalid: return to zero config
                        print('No valid sln, return to zero config')
                        return [0, 0, 0, 0]
                else:
                    if theta < -np.deg2rad(110) or theta > np.deg2rad(110) or math.isnan(theta):
                        all_valid = False
                        # This means both sln are invalid: return to zero config
                        print('No valid sln, return to zero config')
                        return [0, 0, 0, 0]

            
            if all_valid:
                degs = []
                for theta in theta_list:
                    degs.append(np.rad2deg(theta))
                return degs


def main():
    print(inverse_kinematics(100, 100, 0))

if __name__ == '__main__':
    main()