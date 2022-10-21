#!/usr/bin/env python3

#includes stuff
from concurrent.futures.process import _chain_from_iterable_of_lists
from tkinter import NONE
from xml.dom.expatbuilder import parseString
from click import password_option
from gpg import Data
from matplotlib.pyplot import subplot
import numpy as np
import time
import pigpio
from soupsieve import select
import rospy
# import inverse_kinematics as inkin
from inverse_kinematics import *

from std_msgs.msg import Header, String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

centre_x = 0
centre_y = 0
red_x = -50
red_z = -140
green_x = -150
green_z = -50
yellow_x = -150
yellow_z = 50
blue_x = -50
blue_z = 140

rpi = pigpio.pi()
rpi.set_mode(18, pigpio.OUTPUT)

block_list = []
selected_block = 0
getting_colour = False
getting_pos = False


#can ignore z offset because the camera is wonky and we just set it at a specific height each time
Mcr = np.array([[0,1,0,0],
                [0,0,-1,0],
                [-1,0,0,0],
                [0,-0.21*140/30,0,1]]).T

    #Gripper code 

def grip_close():
    rpi.set_servo_pulsewidth(18, 1000)
    return 0

def grip_open():
    rpi.set_servo_pulsewidth(18, 2000)
    return 0   

def grip_box():
    rpi.set_servo_pulsewidth(18, 1300)
    return 0

class Block:
    def __init__(self, x, y, z, block_id):
        self.x = x 
        self.y = y
        self.z = z
        self.block_id = block_id
        self.theta = self.get_theta()
        self.r = np.sqrt((self.x-centre_x)**2+(self.y-centre_y)**2)
        self.colour =None



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
        print("updating position")
        print(self.get_block_id())
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
        wait_time = 3
        hover_height = 32
        self.y = 36
        if np.sqrt(self.x **2 + self.z**2)> 212.5:
            wait_time = 4
            hover_height = 100
            self.y = 51

        grip_open()

        #Preventing floor and block collision
        rospy.loginfo('Grab box')
        pose = Pose()
        pose.position.x = self.x
        pose.position.y = self.y + hover_height
        pose.position.z = self.z
        self.pub.publish(pose)
        print("hovering")
        time.sleep(wait_time)
        # grip_box()

        pose.position.y = self.y
        #start at ready pos 
        self.pub.publish(pose)
        print("Grab")
        print("Moving down")
        time.sleep(2)
        grip_box()

        pose.position.y = self.y + hover_height
        #start at ready pos 
        self.pub.publish(pose)
        print("Moving up")
        time.sleep(1)

    def set_colour(self, colour):
        colour = str(colour).split('"')
        if len(colour) < 2:
            time.sleep(0.1)
            print('waiting')
        else:
            print(colour)
            colour = colour[1]
            self.colour = str(colour)

    def dropoff_box(self):
        #chooses drop off position based on colour
        #dropoff 1 = red
        #dropoff 2 = green
        #dropoff 3 = yellow
        #dropoff 4 = blue
        #self.colour = 'red'
        #print(self.colour)

        pose = Pose()
        if self.colour == 'red':
            pose.position.x = red_x
            pose.position.z = red_z

        elif self.colour == 'green':
            pose.position.x = green_x
            pose.position.z = green_z

        elif self.colour == 'yellow':
            pose.position.x = yellow_x
            pose.position.z = yellow_z

        elif self.colour == 'blue':
            pose.position.x = blue_x
            pose.position.z = blue_z

        else:
            return(1)

        pose.position.y = 100

        self.pub.publish(pose)
        time.sleep(3)
        
        pose.position.y = 36

        self.pub.publish(pose)
        time.sleep(2)

        grip_open()
        time.sleep(1)




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


def get_colour(data):
    global selected_block
    global getting_colour
    if getting_colour != True:
        pass
    else:
        selected_block.set_colour(data)
        #     print('somethig')
        #     self.tester_pub.publish(self.colour)
        # pass



def transform(x, y, z):
    #get point in robot frame and convert to mm from m
    Pc = np.array([x,y,z,1])
    Pr = np.dot(np.linalg.inv(Mcr),Pc)
    #also multiply by scaling factor because camera sucks
    robot_frame_x = (Pr[0] * 1000) * 30/140
    #robot_frame_y = Pr[1] * 100
    #block centre will always be 16mm off the ground and camera height finding is very poor
    robot_frame_y = 36
    robot_frame_z = (Pr[2] * 1000) * 30/140         #was 30/140, changed to mess around a bit
    #robot_frame_theta = 

    return robot_frame_x, robot_frame_y, robot_frame_z 



    #sample 






    #def ready position 


    #def find dropoff zone 

def reset() :
    pose = Pose()
    pose.position.x = 0
    pose.position.y = 0
    pose.position.z = 0
    #start at ready pos 
    rospy.loginfo(f"sending desired pose {pose.position}")
    reset_pub.publish(pose)
    time.sleep(2)


    return 0


def get_camera_list(data : String): 
    global getting_pos
    if getting_pos != True:
        return 1

    global block_list
    #rospy.loginfo("in get camera list")
    camera_list_string = str(data).split('"')
    camera_list_string = str(camera_list_string[1])
    camera_list_string = str(camera_list_string).split(',')
    camera_list = []
    for i in camera_list_string:
        camera_list.append(float(i))
    # print(camera_list[0])
    curr_id = camera_list[0]
    match = False
    print(match)
    curr_block = None
    for b in block_list :
        block_id = b.get_block_id()
        if block_id == curr_id:
            match = True
            print(match)
            curr_block = b
    if match:
        #rospy.loginfo("ID in id_list")
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
    global tester_pub
    global show_cam_pub
    # Initialise node with any node name
    rospy.init_node('logic')
    rospy.loginfo("initialised node")
    # # Create publisher
    # tester_pub = rospy.Publisher(
    #      'tester', # Topic name
    #      String, # Message type
    #     queue_size=10 # Topic size (optional)
    #)
    # test = rospy.Publisher('tester', String, queue_size=10)
    # Create subscriber

    reset_pub = rospy.Publisher(
        'desired_pose', # Topic name
        Pose,
        queue_size=10 # Message type
        )

    colour_sub= rospy.Subscriber(
        'colour_string', # Topic name
        String, # Message type
        get_colour # Callback function (required)
        ) 

    show_cam_pub = rospy.Publisher(
        'desired_joint_states', # Topic name
        JointState, # Message type
        queue_size=10 # Topic size (optional)
    )

    tester_pub = rospy.Publisher(
        'tester', # Topic name
        String, # Message type
        queue_size=10 # Topic size (optional)
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
    


    curr_state = 0
    global selected_block
    global block_list
    global getting_colour
    global getting_pos
    loopstate1 = 0
    while(1):
        print("STARTING LOOP")
        getting_pos = True
        if curr_state == 0 :
            print('State 0')
            #decision making
            if loopstate1 == 0:
                loopstate1 = 1
                reset()

            else:
                time.sleep(1)
                getting_pos = True
                print(block_list)
                if block_list == []:
                
                    continue
                print('State 0')
                selected_block = block_list[0]
                curr_state += 1 

                    
        # Grabbing block
        if curr_state == 1 :
            getting_pos = False
            print("State 1")
            if selected_block == 0:
                print("ERROR: state == 0")
                curr_state = 0
                loopstate1 = 0
                
                break

            selected_block.grab_box()

            curr_state += 1

        # Show camera block colour
        if curr_state == 2 :
            getting_pos = False
            print("State 2")
            
            msg = JointState(
                # Set header with current time
                header=Header(stamp=rospy.Time.now()),
                # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
                name=['joint_1', 'joint_2', 'joint_3', 'joint_4']
            )
            theta_list_mid = [0, 0, np.deg2rad(-66), np.deg2rad(10)]
            msg.position = theta_list_mid
            show_cam_pub.publish(msg)
            time.sleep(2)
            theta_list = [0, np.deg2rad(-40), np.deg2rad(-59), np.deg2rad(-105)]
            print(theta_list)
            msg.position = theta_list

            show_cam_pub.publish(msg)
            time.sleep(3.5)
            getting_colour = True
            time.sleep(0.75)
            getting_colour = False
            
            curr_state += 1 


        #depositing block into drop off zones based on colour
        if curr_state == 3 :
            getting_pos = False
            print("state 3")
            
            selected_block.dropoff_box()
            block_list.remove(selected_block)

            loopstate1 = 0
            curr_state = 0

    rospy.spin()

if __name__ == "__main__":
    main()    
