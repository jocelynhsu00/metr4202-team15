"""
Inverse kinematics
"""

# Always need this

import math

import numpy as np


def inverse_kinematics(x, y, z):
    global pub 

    # INVERSE KINEMATICS
    
    # mm link lengths (from cad)
    l1 = 75 
    l2 = 117.5 
    l3 = 95 
    l4 = 85 

    # End eff posiiton
    x_end = x 
    x_end_abs = abs(x)
    y_end = y- l1
    z_end = z 

    dist_from_centre = np.sqrt(x_end **2 + z_end **2)

    # Prevent self collision and hitting ground
    if (x_end > 80 or x_end < -45) and y_end > -50:

        print("Passed floor collision and env collision check")

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

        # Try gripper 90 deg sln first
        if gripper_90_valid:
            psi = np.pi/2

            # Find pos at dynamixel 4
            x_4 = x_end_abs
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
            # Check limits
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
                print("Using 90 deg")

        # Try gripper 0 pos
        if (all_valid is None or all_valid is False) and gripper_0_valid:
            print("Trying 0")
            # Try gripper 0 position
            psi = 0

            # Find pos at dynamixel 4
            x_4 = x_end_abs - l4
            y_4 = y_end
            z_4 = z_end

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

            theta_list  = [theta_1, theta_2, theta_3, theta_4]
            # Check limits
            all_valid = True
            for index, theta in enumerate(theta_list):
                # Check dynamixel limits (want to avoid going over -90, 90 deg as it damages the wiring)
                if index != 3:
                    if theta < -np.deg2rad(140) or theta > np.deg2rad(140) or math.isnan(theta):
                        all_valid = False
                        # This means both sln are invalid: return to zero config
                        print('No valid sln, ignore')
                        break
                else:
                    if theta < -np.deg2rad(110) or theta > np.deg2rad(110) or math.isnan(theta):
                        all_valid = False
                        # This means both sln are invalid: return to zero config
                        print('No valid sln, return to zero config')
                        break
            
            if all_valid:
               print("Using 0 deg")

        deg = []
        for theta in theta_list:
            deg.append(np.rad2deg(theta))
        print(deg)


def main():
   inverse_kinematics(250, 40, 50)

if __name__ == '__main__':
    main()