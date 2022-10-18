#!/usr/bin/env python3

#includes stuff
from concurrent.futures.process import _chain_from_iterable_of_lists
from matplotlib.pyplot import subplot
import numpy as np
import time
import pigpio
import rospy
# import inverse_kinematics as inkin
from inverse_kinematics import *

from std_msgs.msg import Header, String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

centre_x = 0
centre_y = 0

rpi = pigpio.pi()
rpi.set_mode(18, pigpio.OUTPUT)

id_list = []

class Block:
    def __init__(self, x, y, z):
        self.x = x 
        self.y = y
        self.z = z
        self.theta = self.get_theta()
        self.r = np.sqrt((self.x-centre_x)**2+(self.y-centre_y)**2)

    def update_Pos(self, x, y, z):
        #Update a block opjects current position.
        #maybe update r. 
        self.x = x
        self.y = y
        self.z = z 
        self.theta = self.get_theta()
        self.r = np.sqrt((self.x-centre_x)**2+(self.y-centre_y)**2)
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
        return pred_x, pred_y
        
        
def block_distance(b1,b2):
    d = np.sqrt((b1.x-b2.x)**2+(b1.y-b2.y)**2)
    return(d)








#def transform(x, y, z, theta):
   #return 0 



#sample 



#Gripper code 

def grip_close():
    rpi.set_servo_pulsewidth(18, 1000)
    return 0

def grip_open():
    rpi.set_servo_pulsewidth(18, 2000)
    return 0

def grip_box():
    rpi.set_servo_pulsewidth(18, 1500)
    return 0


#def ready position 

def grab_box(block) :
    pose = Pose 
    pose.position.x = 100
    pose.position.y = 100
    pose.position.z = 0
    #start at ready pos 
    inverse_kinematics(pose)
    publish()
    #block predict pos 
    pose.position.x, pose.position.y = block.predict_pos()
    pose.position.y = pose.position.y + 20
    #move to predicted pos 
    inverse_kinematics(pose)
    publish()
    #maybe wait for box to move underneath
    #move down 
    pose.position.y = pose.position.y - 20
    inverse_kinematics(pose)
    publish()
    #gripbox
    grip_box()
    #move up
    pose.position.y = pose.position.y + 20
    inverse_kinematics(pose)
    publish()
    #move to dropoff zone
    pose.position.x = 0
    pose.position.y = 0
    pose.position.z = 0
    #gripopen
    grip_open()
    #reset to ready position \
    return 0 
    

#def find dropoff zone 

def reset() :
    # Create message of type JointState
    msg = Pose(
        # Set header with current time
        #header=Header(stamp=rospy.Time.now()),
        # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
        #name=['joint_1', 'joint_2', 'joint_3', 'joint_4']
    )

    #publish()
    pose = Pose
    #print(pose) 
    #print(msg)
    # pose.position.y = 100
    # pose.position.z = 0
    # #start at ready pos 
    # inverse_kinematics(pose)
    # publish()
    # return 0

# def pub():
#     global pub

#     pub = rospy.Publisher('desired_pose', Pose, reset)
 
def get_camera_list(data : String):
    camera_list_string = str(data).split('"')
    camera_list_string = str(camera_list_string[1])
    camera_list_string = str(camera_list_string).split(',')
    camera_list = []
    for i in camera_list_string:
        camera_list.append(float(i))

    if camera_list[0] in id_list:
        #use update_pos()
        camera_list[0].update_pos(camera_list[1], camera_list[2], camera_list[3])
    else:
        id_list.append(camera_list[0])
        camera_list[0] = Block(camera_list[1], camera_list[2], camera_list[3])
        pub.publish(str(id_list))
    


def main():
    global pub
    # # Create publisher
    # pub = rospy.Publisher(
    #     'desired_joint_states', # Topic name
    #     JointState, # Message type
    #     queue_size=10 # Topic size (optional)
    # )
    pub = rospy.Publisher('tester', String, queue_size=10)
    # Create subscriber
    sub = rospy.Subscriber(
        'camera_list', # Topic name
        String, # Message type
        get_camera_list # Callback function (required)
    )


    # Initialise node with any node name
    rospy.init_node('joint_states')

    # You spin me right round baby, right round...
    # Just stops Python from exiting and executes callbacks
    rospy.spin()

if __name__ == "__main__":
    main()