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

from std_msgs.msg import Header, String, Int16
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

# Drop off zone positions
centre_x = 0
centre_y = 0
yeet_x = 200
yeet_z = 100
yeet_x_throw = -200
yeet_z_throw = -100

yeet_blue = -16.3
yeet_green = 30
yeet_yellow = -30
yeet_red = 16.3

#list length for x and z stop. 
x_z_list_length = 5


# Set pwm for servo
rpi = pigpio.pi()
rpi.set_mode(18, pigpio.OUTPUT)

# Initialising block list
block_list = []

# Initialising selected block
selected_block = 0

# Intialised color detection, not reading colour when False
getting_colour = False

# Initialised position detection (aruco tag), not updating position when False
getting_pos = False

# Camera transform
# setting specific height for z to avoid inaccurate values from camera
Mcr = np.array([[0,1,0,0],
                [0,0,-1,0],
                [-1,0,0,0],
                [0,-0.21*140/30,0,1]]).T

# Gripper

def grip_close():   
    """
    Closes the gripper
    """
    rpi.set_servo_pulsewidth(18, 1000)
    return 0

def grip_open():
    """
    Opens the gripper
    """
    rpi.set_servo_pulsewidth(18, 2000)
    return 0   

def grip_block():
    """
    Closes the gripper to gripping block position
    """
    rpi.set_servo_pulsewidth(18, 1300)
    return 0


def get_colour(data):
    """
    Get colour based on camera input
    """
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
    """
    Frame transform
    """
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


def reset():
    """
    Reset position
    """
    # Ikin will go to reset pos when invalid pose is given
    pose = Pose()
    pose.position.x = 0
    pose.position.y = 0
    pose.position.z = 0
    #start at ready pos 
    rospy.loginfo(f"sending desired pose {pose.position}")
    reset_pub.publish(pose)
    time.sleep(1)
    return 0

def get_gripper_pos(data: String):
    """
    Subscribes to gripper_pos topic to determine if 90 or 0 is used
    """
    global selected_block
    gripper_pos_list = str(data).split('"')
    gripper_pos= int(gripper_pos_list[1])
    selected_block.set_gripper_pos(gripper_pos)
    


class Block:
    """
    Class to manage the pickup and dropoff of blocks
    """
    def __init__(self, x, y, z, block_id):
        """
        Constructor
        
        Parameters:
            x: The x position of the centre of the block
            y: The y position of the centre of the block
            z: The z position of the centre of the block
            block_id: The aruco tag id for the given block
        
        Returns:
            None
        """
        self.x = x 
        self.y = y
        self.z = z
        self.xlist = []
        self.zlist = []
        self.block_id = block_id
        self.theta = self.get_theta()
        self.r = np.sqrt((self.x-centre_x)**2+(self.y-centre_y)**2)
        self.colour =None
        self.gripper_pos = -1

        self.is_stationary = False

        # Initialise Pose publisher (ikin subscribes to this)
        self.pub = rospy.Publisher(
            'desired_pose', # Topic name
            Pose,
            queue_size=10 # Message type
        )

    def get_block_id(self):
        """
        Gets the block id
        """
        return self.block_id

    def update_pos(self, x, y, z):
        """
        Updates the block objects current position

        Parameters:
            x: The x position of the centre of the block
            y: The y position of the centre of the block
            z: The z position of the centre of the block
        
        Returns:
            None
        """
        self.x = x
        self.y = y
        self.z = z 
        self.theta = self.get_theta()
        self.r = np.sqrt((self.x-centre_x)**2+(self.y-centre_y)**2)
        print("Updating position")
        print(self.get_block_id())
        return 0
    
    def set_gripper_pos(self, gripper_pos):
        self.gripper_pos = gripper_pos

    def get_gripper_pos(self):
        return self.gripper_pos

    def get_pos(self):
        """
        Gets the blocks current position
        """
        return self.x, self.y, self.z
    
    def get_theta(self):
        """
        Calculates the angle of the block to use for angular velocity calculation
        Starts on the right, moves counter clockwise
        """
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
        """
        Use two theta values to get the angular velocity of the block
        """
        theta1 = self.get_theta()
        time.sleep(3)
        theta2 = self.get_theta()
        delta_theta = theta2 - theta1
        omega = delta_theta/3
        return omega
    
    def predict_pos(self):
        """
        Position prediction
        """
        pred_theta = self.theta+self.omega*3
        pred_y = np.arcsin(pred_theta)*self.r
        pred_x = np.arccos(pred_theta)*self.r
        return pred_x, pred_y
        
        
    def block_distance(b1,b2):
        """
        Calculate the block distance
        """
        d = np.sqrt((b1.x-b2.x)**2+(b1.y-b2.y)**2)
        return(d)


    # def grab_block(self):
    #     """
    #     Grabbing the block 
    #     """


    #     # Open gripper
    #     grip_open()

    #     # Hover above block
    #     # Preventing floor and block collision
    #     rospy.loginfo('Grab box')
    #     pose = Pose()
    #     pose.position.x = self.x
    #     pose.position.y = 100
    #     pose.position.z = self.z

    #     self.pub.publish(pose)

    #     if self.get_gripper_pos() == 90:
    #         # Update wait time, hover height and y position
    #         wait_time = 3  #4
    #         self.y = 36
    #     else:
    #         wait_time = 4 #3
    #         self.y = 51
    #     #go for actual block after knowing what pos it is using
    #     pose.position.y = self.y
    #     time.sleep(wait_time)
    #     self.pub.publish(pose)
        
    #     time.sleep(4)
    #     grip_block()
        

    #     # Move up with block
    #     pose.position.y = self.y + 100
    #     #start at ready pos 
    #     self.pub.publish(pose)
    #     print("Moving up")
    #     time.sleep(1)

    def grab_block(self):
        """
        Grabbing the block 
        """


        # Open gripper
        grip_open()

        # Hover above block
        # Preventing floor and block collision
        rospy.loginfo('Grab box')
        pose = Pose()
        pose.position.x = self.x
        #pose.position.y = 100
        pose.position.z = self.z

        #self.pub.publish(pose)

        # if self.get_gripper_pos() == 90:
        #     # Update wait time, hover height and y position
        #     wait_time = 3  #4
        #     self.y = 36
        wait_time = 4 #3
        self.y = 51
        #go for actual block after knowing what pos it is using
        pose.position.y = self.y
        #time.sleep(wait_time)
        self.pub.publish(pose)
        time.sleep(1.5)
        if self.get_gripper_pos() == 90:
            pose.position.y = 36
            self.pub.publish(pose)
            time.sleep(0.5)
        
        grip_block()
        
        

        # Move up with block
        pose.position.y = self.y + 100
        #start at ready pos 
        self.pub.publish(pose)
        print("Moving up")
        time.sleep(0.5)

    def set_colour(self, colour):
        """
        Sets the color of the block
        """
        global getting_colour
        colour = str(colour).split('"')
        if len(colour) < 2:
            #time.sleep(0.1)
            print('waiting')
        else:
            getting_colour = False
            print(colour)
            colour = colour[1]
            self.colour = str(colour)

    def yeet_box(self):
        """
        Throws a block based on colour
        """

        msg = JointState(
        # Set header with current time
        header=Header(stamp=rospy.Time.now()),
        # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
        name=['joint_1', 'joint_2', 'joint_3', 'joint_4']
            )

        # Create a pose object

        # Set pose based on colour
        theta_1 = 0
        pose = Pose()

        if self.colour == 'red':
            theta_1 = yeet_red

        elif self.colour == 'green':
            theta_1 = yeet_green

        elif self.colour == 'yellow':
            theta_1 = yeet_yellow

        elif self.colour == 'blue':
            theta_1 = yeet_blue
        else:
            return(1)

        theta_list_mid = [np.deg2rad(theta_1), np.deg2rad(-90), 0, 0]
        msg.position = theta_list_mid
        show_cam_pub.publish(msg)
        time.sleep(3)

        theta_list_mid = [np.deg2rad(-30), np.deg2rad(-45), np.deg2rad(45), 0]
        msg.position = theta_list_mid
        show_cam_pub.publish(msg)
        time.sleep(0.4)
        grip_open()
        time.sleep(3)
    def set_stationary(self, stationary: bool):
        self.is_stationary = stationary

    def get_stationary(self):
        return self.is_stationary

def get_camera_list(data : String): 
    """
    Get camera information from camera publisher
    """
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


        if len(curr_block.xlist) >= x_z_list_length:
            curr_block.xlist.remove(curr_block.xlist[0])
        curr_block.xlist.append(x)

        if len(curr_block.zlist) >= x_z_list_length:
            curr_block.zlist.remove(curr_block.zlist[0])
        curr_block.zlist.append(z)


        # checks if they are similar so it knows its stopped
        if len(curr_block.xlist) >= x_z_list_length:
            x_diff = sum(curr_block.xlist)/x_z_list_length
            z_diff = sum(curr_block.zlist)/x_z_list_length



        # checks to make sure diff of each is less than error_diff
            error_diff = 1.02
            if abs(x) <= abs(x_diff) * error_diff:
                print('no difference in x')
                if abs(z) <= abs(z_diff) * error_diff:
                    print('no difference in z')
                    curr_block.set_stationary(True)
                else:
                    curr_block.set_stationary(False)
            else:
                curr_block.set_stationary(False)


        curr_block.update_pos(x, y, z)
        print("updated existing block position")


    else:
        rospy.loginfo("else")
        #need to transform frame first
        pos = transform(camera_list[1], camera_list[2], camera_list[3])
        x = pos[0]
        y = pos[1]
        z = pos[2]
        #for finding if stopped

        curr_block = Block(x, y, z, curr_id)
        block_list.append(curr_block)
        curr_block.listx = [x]
        curr_block.listz = [z]
        print("created new block")
        # id_list[0].grab_block()
    #test.publish(str(id_list))

def main():
    """
    Main method
    """
    # Setup
    global reset_pub
    global show_cam_pub
    global selected_block
    global block_list
    global getting_colour
    global getting_pos

    
    loop_state_1 = 0
    curr_state = 0

    # Initialise node
    rospy.init_node('logic')
    rospy.loginfo("initialised logic node")

    # Create publishers (used outside of Block class)
    reset_pub = rospy.Publisher(
        'desired_pose', # Topic name
        Pose,
        queue_size=10 # Message type
    )

    show_cam_pub = rospy.Publisher(
        'desired_joint_states', # Topic name
        JointState, # Message type
        queue_size=10 # Topic size (optional)
    )

    # Create subscribers
    sub_colour= rospy.Subscriber(
        'colour_string', # Topic name
        String, # Message type
        get_colour # Callback function (required)
    ) 

    sub_cam_list = rospy.Subscriber(
        'camera_list', # Topic name
        String, # Message type
        get_camera_list # Callback function (required)
    )

    sub_gripper_pos = rospy.Subscriber(
        'gripper_pos',
        String,
        get_gripper_pos
    )

 
    # State machine
    while(1):
        print("STARTING STATE MACHINE")
        # print("Gripper pos", gripper_pos)
        #gripper_pub.publish(gripper_pos)

        if curr_state == 0:
            # Determine if block has been detected
            print('State 0')
            if loop_state_1 == 0:
                loop_state_1 = 1
                reset()
                time.sleep(0.5)

            else:
                #time.sleep(1)
                getting_pos = True
                print(block_list)
                if block_list == []:
                
                    continue
                print('State 0')
                selected_block = block_list[0]
                if selected_block.get_stationary():
                    curr_state += 1 

                    
        # Grabbing block
        if curr_state == 1 :
            getting_pos = False
            print("State 1")
            if selected_block == 0:
                print("ERROR: state == 0")
                curr_state = 0
                loop_state_1 = 0
                
                break
            for index, block in enumerate(block_list):
                too_close = False
                for i, b in enumerate(block_list):
                    if i == index:
                        continue
                    distance = np.sqrt(block.r**2+b.r**2-2*block.r*b.r*np.cos(np.abs(block.theta-b.theta)))
                    if distance <= 40 and np.abs(block.r-b.r) <= 32:
                        too_close = True
                if block.r < selected_block.r and too_close == False:
                    selected_block = block

            selected_block.grab_block()
            curr_state += 1

        # Prepare for yeet
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
            #time.sleep(2)
            theta_list = [0, np.deg2rad(-35), np.deg2rad(-55), np.deg2rad(-100)]
            print(theta_list)
            msg.position = theta_list

            show_cam_pub.publish(msg)
            time.sleep(1.5)
            getting_colour = True
            time.sleep(0.1)
            getting_colour = False
            selected_block.yeet_box()
            curr_state += 1 


        # Yeet
        if curr_state == 3 :
            getting_pos = False
            print("state 3")
            
            # selected_block.dropoff_box()
            block_list.remove(selected_block)

            loop_state_1 = 0
            curr_state = 0



    rospy.spin()

if __name__ == "__main__":
    main()    
