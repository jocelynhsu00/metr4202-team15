#includes stuff
import numpy as np
import time
centre_x = 0
centre_y = 0


class Block:
    def __init__(self, x, y, z, theta)
    self.x = x 
    self.y = y
    self.z = z
    self.theta = theta
    self.r = np.sqrt((self.x-centre_x)^2+(self.y-centre_y)^2)

    def update_Pos(self, x, y, z, theta):
        #Update a block opjects current position.
        #maybe update r. 
        return 0 

    def get_Pos(self):
        # Get blocks current position.
        return 0 
    
    def get_theta(self, x, y, z):
        #Calculating theta for the block, starting on the right and going around counterclockwise
        phi = np.absolute(np.arctan((self.y-centre_y)/(self.x-centre_x)))
        if self.x > centre_x:
            if self.y > centre_y:
                theta = phi
            else if self.y < centre_y:
                theta = 2*np.pi - phi
        else if self.x < centre_x:
            if self.y > centre_y:
                theta = np.pi - phi
            else if self.y < centre_y:
                theta = np.pi + phi
        return theta
    
    def get_omega(self):
        #uses two thetas to find the angular velocity of the cube
        theta1 = get_theta
        time.sleep(3)
        theta2 = get_theta
        delta_theta = theta2 - theta1
        omega = delta_theta/3
        return omega
        
        









def transform(x, y, z, theta):
    return 0 



#sample 



# copy gripper code 





