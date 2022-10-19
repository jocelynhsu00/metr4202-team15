#!/usr/bin/env python3

#includes stuff
from concurrent.futures.process import _chain_from_iterable_of_lists
from tkinter import NONE
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

block_list = []

Mcr = np.array([[1,0,0,0],
                [0,1,0,0],
                [0,0,1,0],
                [1,1,1,1]]).T

class Block:
    def __init__(self, x, y, z, block_id):
        self.x = x 
        self.y = y
        self.z = z
        self.block_id = block_id
        self.theta = self.get_theta()
        self.r = np.sqrt((self.x-centre_x)**2+(self.y-centre_y)**2)

        self.pub = rospy.Publisher(
            'desired_pose', # Topic name
            Pose,
            queue_size=10 # Message type
        )

    def get_block_id(self):
        return self.block_id

    def update_pos(self, x, y, z):
        #Update a block opjects current position.
        #maybe update r. 
        self.x = x
        self.y = y
        self.z = z 
        self.theta = self.get_theta()
        self.r = np.sqrt((self.x-centre_x)**2+(self.y-centre_y)**2)
        return 0 

    def get_pos(self):
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


    def grab_box(self):
        #reset()

        #test
        rospy.loginfo('Grab box')
        pose = Pose()
        pose.position.x = self.x
        pose.position.y = self.y
        pose.position.z = self.z
        #start at ready pos 
        self.pub.publish(pose)


        # #block predict pos 
        # pose.position.x, pose.position.y = block.predict_pos()
        # pose.position.y = pose.position.y + 20

        # #move to predicted pos 

        # #maybe wait for box to move underneath

        # #move down 
        # pose.position.y = pose.position.y - 20

        
        # #gripbox
        # grip_box()
        # #move up
        # pose.position.y = pose.position.y + 20
        
        # pub.publish()
        # #move to dropoff zone
        # pose.position.x = 0
        # pose.position.y = 0
        # pose.position.z = 0
        # #gripopen
        # grip_open()
        #reset to ready position \






def transform(x, y, z):
    Pc = np.array([x,y,z,1])
    Pr = np.dot(np.linalg.inv(Mcr),Pc)
    robot_frame_x = Pr[0]
    robot_frame_y = Pr[1]
    robot_frame_z = Pr[2]
    #robot_frame_theta = 

    return robot_frame_x, robot_frame_y, robot_frame_z 



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


    #def find dropoff zone 

def reset() :
    pose = Pose()
    pose.position.x = 110
    pose.position.y = 100
    pose.position.z = 0
    #start at ready pos 
    rospy.loginfo(f"sending desired pose {pose.position}")
    reset_pub.publish(pose)

    return 0


def get_camera_list(data : String): 
    rospy.loginfo("in get camera list")
    camera_list_string = str(data).split('"')
    camera_list_string = str(camera_list_string[1])
    camera_list_string = str(camera_list_string).split(',')
    camera_list = []
    for i in camera_list_string:
        camera_list.append(float(i))
    print(camera_list[0])
    curr_id = camera_list[0]
    match = False
    curr_block = None
    for b in block_list :
        block_id = b.get_block_id()
        if block_id == curr_id:
            match = True
            curr_block = b
    if match:
        rospy.loginfo("ID in id_list")
        #need to transform frame first

        #use update_pos()
        pos = transform(camera_list[1], camera_list[2], camera_list[3])
        x = pos[0]
        y = pos[1]
        z = pos[2]
        curr_block.update_pos(x, y, z)
        print("updated existing block position")


    else:
        rospy.loginfo("else")
        #need to transform frame first
        pos = transform(camera_list[1], camera_list[2], camera_list[3])
        x = pos[0]
        y = pos[1]
        z = pos[2]

        curr_block = Block(x, y, z, curr_id)
        block_list.append(curr_block)
        print("created new block")

        # id_list[0].grab_box()
    #test.publish(str(id_list))
            


def main():
    # global pub
    # global test
    global reset_pub
    # Initialise node with any node name
    rospy.init_node('logic')
    rospy.loginfo("initialised node")
    # # Create publisher
    # pub = rospy.Publisher(
    #     'desired_joint_states', # Topic name
    #     JointState, # Message type
    #     queue_size=10 # Topic size (optional)
    # )
    # test = rospy.Publisher('tester', String, queue_size=10)
    # Create subscriber

    reset_pub = rospy.Publisher(
        'desired_pose', # Topic name
        Pose,
        queue_size=10 # Message type
        )

    
    sub = rospy.Subscriber(
        'camera_list', # Topic name
        String, # Message type
        get_camera_list # Callback function (required)
    )

    rospy.loginfo("subscribing to camera_list")
    # while(1):
    #     if len(id_list) != 0:
    #         id_list[0].grab_box()
    #     else:
    #         print(id_list)
    

    # You spin me right round baby, right round...
    # Just stops Python from exiting and executes callbacks
    rospy.spin()

if __name__ == "__main__":
    main()    
