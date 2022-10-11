#includes stuff
import numpy as np
import time
import pigpio 
centre_x = 0
centre_y = 0

rpi = pigpio.pi()
rpi.set_mode(18, pigpio.OUTPUT)

class Block:
    def __init__(self, x, y, z, theta):
        self.x = x 
        self.y = y
        self.z = z
        self.theta = self.get_theta()
        self.r = np.sqrt((self.x-centre_x)^2+(self.y-centre_y)^2)

    def update_Pos(self, x, y, z, theta):
        #Update a block opjects current position.
        #maybe update r. 
        self.x = x
        self.y = y
        self.z = z 
        self.theta = self.get_theta()
        self.r = np.sqrt((self.x-centre_x)^2+(self.y-centre_y)^2)
        return 0 

    def get_Pos(self):
        # Get blocks current position.
        return self.x, self.y, self.z
    
    def get_theta(self):
        #Calculating theta for the block, starting on the right and going around counterclockwise
        phi = np.absolute(np.arctan((self.y-centre_y)/(self.x-centre_x)))
        if self.x > centre_x:
            if self.y > centre_y:
                theta = phi
            elif self.y < centre_y:
                theta = 2*np.pi - phi
        elif self.x < centre_x:
            if self.y > centre_y:
                theta = np.pi - phi
            elif self.y < centre_y:
                theta = np.pi + phi
        return theta
    
    def get_omega(self):
        #uses two thetas to find the angular velocity of the cube
        theta1 = self.get_theta()
        time.sleep(3)
        theta2 = self.get_theta()
        delta_theta = theta2 - theta1
        omega = delta_theta/3
        return omega
    
    def predict_pos(self):
        pred_theta = self.theta+self.omega*3
        pred_y = np.arcsin(pred_theta)*self.r
        pred_x = np.arccos(pred_theta)*self.r
        
        
def block_distance(b1,b2):
    d = np.sqrt((b1.x-b2.x)^2+(b1.y-b2.y)^2)
    return(d)








def transform(x, y, z, theta):
    return 0 



#sample 



# copy gripper code 

def grip_close():
    rpi.set_servo_pulsewidth(18, 1000)
    return 0

def grip_open():
    rpi.set_servo_pulsewidth(18, 2000)
    return 0

def grip_box():
    rpi.set_servo_pulsewidth(18, 1500)
    return 0




