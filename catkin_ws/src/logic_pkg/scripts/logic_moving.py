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
centre_x = 190
centre_z = 0
red_x = -41
red_z = -140
green_x = -140
green_z = -50
yellow_x = -140
yellow_z = 50
blue_x = -41
blue_z = 140

# List length for x and z position collection
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
# Setting specific height for z to avoid inaccurate values from camera
Mcr = np.array([[0,1,0,0],
                [0,0,-1,0],
                [-1,0,0,0],
                [0,-0.21*140/30,0,1]]).T

# Gripper

def grip_close():   
    """
    Closes the gripper

    Parameters:
        None

    Returns:
        None
    """
    rpi.set_servo_pulsewidth(18, 1000)
    return 0

def grip_open():
    """
    Opens the gripper

    Parameters:
        None

    Returns:
        None
    """
    rpi.set_servo_pulsewidth(18, 2000)
    return 0   

def grip_block():
    """
    Closes the gripper to gripping block position

    Parameters:
        None

    Returns:
        None
    """
    rpi.set_servo_pulsewidth(18, 1300)
    return 0


def get_colour(data):
    """
    Get colour based on camera input

    Parameters:
        data(String): colour string

    Returns:
        None
    """
    global selected_block
    global getting_colour
    if getting_colour != True:
        pass
    else:
        selected_block.set_colour(data)


def transform(x, y, z):
    """
    Frame transform

    Parameters:
        x(int): the x value from the camera frame
        y(int): the y value from the camera frame
        z(int): the z value from the camera frame

    Returns:
        (int, int, int): the transformed x, y, z to the robot frame
    """
    # Get point, convert from m to mm
    Pc = np.array([x,y,z,1])
    Pr = np.dot(np.linalg.inv(Mcr),Pc)
    # Scaling factor
    robot_frame_x = (Pr[0] * 1000) * 30/140
    # Tested with ikin 36 is the correct height to grab block given extended gripper
    robot_frame_y = 36
    robot_frame_z = (Pr[2] * 1000) * 30/140 

    return robot_frame_x, robot_frame_y, robot_frame_z 


def reset():
    """
    Reset position

    Parameters:
        None

    Returns:
        (int): 0 for success
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

    Parameters:
        data(String): the string given from the gripper pos topic subscriber

    Returns:
        None
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
        self.xlist = [1,2,3,4,5]
        self.zlist = [1,2,3,4,5]
        self.tlist = [1,2,3,4,5]
        self.block_id = block_id
        self.theta = self.get_theta()
        self.r = np.sqrt((self.x-centre_x)**2+(self.z-centre_z)**2)
        self.colour =None
        self.gripper_pos = -1
        self.dont_get = False
        self.is_stationary = False
        self.clockwise = True

        # Initialise Pose publisher (ikin subscribes to this)
        self.pub = rospy.Publisher(
            'desired_pose', # Topic name
            Pose,
            queue_size=10 # Message type
        )

    def get_block_id(self):
        """
        Gets the block id

        Parameters:
            None

        Returns;
            (int): block id
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
        self.r = np.sqrt((self.x-centre_x)**2+(self.z-centre_z)**2)
        return 0
    
    def set_gripper_pos(self, gripper_pos):
        """
        Set the gripper position based on i kin publisher
        
        Parameters:
            gripper_pos(int): 0, 90 or -1
            
        Returns:
            None
        """
        self.gripper_pos = gripper_pos

    def get_gripper_pos(self):
        """
        Get the gripper position
        
        Parameters:
            None
            
        Returns:
            (int): gripper position
        """
        return self.gripper_pos

    def get_dont_get(self):
        """
        Do not try to pickup block
        
        Parameters:
            None
            
        Returns:
            (bool): True if the block should not be picked up
        """
        
        return self.dont_get


    def set_dont_get(self, statement: bool):
        """
        Tries to pickup block
        
        Parameters:
            statement(bool): True or False for get block or dont get block
            
        Returns:
            None
        """
        self.dont_get = statement


    def get_pos(self):
        """
        Gets the blocks current position

        Parameters:
            None

        Returns:
            (int, int, int): x, y, z
        """
        return self.x, self.y, self.z
    
    def get_theta(self):
        """
        Calculates the angle of the block to use for angular velocity calculation
        Starts on the right, moves counter clockwise

        Parameters:
            None

        Returns:
            (int): theta
        """
        phi = np.absolute(np.arctan((self.z-centre_z)/(self.x-centre_x)))
        if self.x > centre_x:
            if self.z > centre_z:
                theta = phi
            elif self.z < centre_z:
                theta = 2*np.pi - phi
        elif self.x < centre_x:
            if self.z > centre_z:
                theta = np.pi - phi
            elif self.z < centre_z:
                theta = np.pi + phi
        
        return theta
    
    def get_omega(self):
        """
        Use two theta values to get the angular velocity of the block

        Parameters:
            None

        Returns:
            (int): omega

        """

        phi = np.absolute(np.arctan((self.zlist[3] -centre_z)/(self.xlist[3]-centre_x)))
        if self.x > centre_x:
            if self.z > centre_z:
                theta2 = phi
            elif self.z < centre_z:
                theta2 = 2*np.pi - phi
        elif self.x < centre_x:
            if self.z > centre_z:
                theta2 = np.pi - phi
            elif self.z < centre_z:
                theta2 = np.pi + phi

        theta1 = self.get_theta()

        delta_theta = theta2 - theta1
        omega = delta_theta/(self.tlist[4] - self.tlist[3])
        return omega
    
    def predict_pos(self):
        """
        Position prediction

        Parameters:
            None

        Returns:
            (int, int): x, y
        """
        pred_theta = self.theta+self.omega*3
        pred_y = np.arcsin(pred_theta)*self.r
        pred_x = np.arccos(pred_theta)*self.r
        return pred_x, pred_y
        
        
    def block_distance(b1,b2):
        """
        Calculate the block distance

        Parameters:
            b1(Block): block object
            b2(Block): block object

        Returns:
            (int): distance between 2 blocks
        """
        d = np.sqrt((b1.x-b2.x)**2+(b1.y-b2.y)**2)
        return(d)


    def grab_block(self):
        """
        Grabbing the block 

        Parameters:
            None

        Returns:
            None
        """
        # Open gripper
        global selected_block
        grip_open()

        omega = self.get_omega()

        #radius = abs(self.r)
        if omega < 0:
            radius = self.r
        else:
            radius = -self.r

        # Send end effector position to inv kin
        rospy.loginfo('Grab box')
        pose = Pose()
        pose.position.x = centre_x
        pose.position.z = radius
        wait_time = 4 
        self.y = 51

        # Move to position
        pose.position.y = self.y
        self.pub.publish(pose)

        # Give robot time to move to pose
        time.sleep(wait_time)
        
        # Close gripper
        grip_block()
        time.sleep(0.1)
        

    def set_colour(self, colour):
        """
        Sets the color of the block

        Parameters:
            colour(string): the string containing the colour

        Returns:
            None
        """
        global getting_colour
        # Split the string
        colour = str(colour).split('"')

        # Didnt get valid colour
        if len(colour) < 2:
            rospy.loginfo('Waiting')
        else:
            getting_colour = False
            colour = colour[1]
            # Update colour
            self.colour = str(colour)

    def dropoff_box(self):
        """
        Drops of block based on colour

        Parameters:
            None

        Returns:
            None
        """
        # Create a pose object
        # Set pose based on colour
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
        
        # Move to dropoff position
        pose.position.y = 36

        self.pub.publish(pose)
        time.sleep(1.3)

        # Drop off block
        grip_open()
        time.sleep(0.3)

    def set_stationary(self, stationary: bool):
        """
        Set if the block is stationary
        
        Parameters:
            stationary(bool): True if block is stationary, False if not
            
        Returns:
            None
        """
        self.is_stationary = stationary

    def get_stationary(self):
        """
        Get if the block is stationary
        
        Parameters;
            None
            
        Returns:
            (bool): True if the block is stationary, False if its not
        """
        return self.is_stationary

def get_camera_list(data : String): 
    """
    Get camera information from camera publisher

    Parameters:
        data(String): the string from the camera publisher
    """
    global getting_pos
    if getting_pos != True:
        return 1

    global block_list
    # Split the string
    camera_list_string = str(data).split('"')
    camera_list_string = str(camera_list_string[1])
    camera_list_string = str(camera_list_string).split(',')
    camera_list = []

    # Update camera list
    for i in camera_list_string:
        camera_list.append(float(i))
    curr_id = camera_list[0]

    # Check for match
    match = False
    curr_block = None

    # Check for match in block list
    for b in block_list :
        block_id = b.get_block_id()

        # Get the current block if id matches
        if block_id == curr_id:
            match = True
            curr_block = b

    if match:
        # Transform the data
        pos = transform(camera_list[1], camera_list[2], camera_list[3])
        x = pos[0]
        y = pos[1]
        z = pos[2]
        t = time.time()

        # Check the block position 5 times to determine if it is stationary
        if len(curr_block.xlist) >= x_z_list_length:
            curr_block.xlist.remove(curr_block.xlist[0])
        curr_block.xlist.append(x)

        if len(curr_block.zlist) >= x_z_list_length:
            curr_block.zlist.remove(curr_block.zlist[0])
        curr_block.zlist.append(z)

        if len(curr_block.tlist) >= 5:
            curr_block.tlist.remove(curr_block.tlist[0])
        curr_block.tlist.append(t)

        # Checks to make sure the distance difference is outside of expected error
        error_diff = 1.02
        if 180 < curr_block.get_theta() < 360:
            curr_block.set_dont_get(True)
        else:
            curr_block.set_dont_get(False)

        # Update block position
        curr_block.update_pos(x, y, z)
        rospy.loginfo("Updating block position")

    else:
        rospy.loginfo("else")
        # Need to transform frame first
        pos = transform(camera_list[1], camera_list[2], camera_list[3])
        x = pos[0]
        y = pos[1]
        z = pos[2]

        curr_block = Block(x, y, z, curr_id)
        block_list.append(curr_block)
        curr_block.listx = [x]
        curr_block.listz = [z]
        rospy.loginfo("created new block")

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

        # DETECTION STATE
        if curr_state == 0:
            print('State 0')
            # Check how many times robot has reset
            if loop_state_1 == 0:
                loop_state_1 = 1
                reset()
                time.sleep(0.5)

            else:
                # Keep looking for block position
                getting_pos = True
                # If block list is empty, wait for a new block
                if block_list == []:
                    continue
                selected_block = block_list[0]
                # Move onto grab block if the block is stationary
                curr_state += 1 


        # GRAB BLOCK STATE
        if curr_state == 1:
            print("State 1")

            for block in block_list :
                if block.get_theta() < selected_block.get_theta() :
                    selected_block = block

            # Do not update position while grabbing block
            getting_pos = False
            if selected_block == 0:
                print("ERROR: state == 0")
                curr_state = 0
                loop_state_1 = 0
                break

            # Check if blocks are too close together
            # for index, block in enumerate(block_list):
            #     too_close = False
            #     for i, b in enumerate(block_list):
            #         if i == index:
            #             continue
            #         # Check distance between blocks
            #         distance = np.sqrt(block.r**2+b.r**2-2*block.r*b.r*np.cos(np.abs(block.theta-b.theta)))

            #         # Confirm is distance is big enough to grab block
            #         if distance <= 40 and np.abs(block.r-b.r) <= 32:
            #             too_close = True
                
            #     # Grab the closest valid block
            #     if block.r < selected_block.r and too_close == False:
            #         selected_block = block
            

            #Iterate to find block closest to 90 degrees

    
            # Grab block and move onto next state
            if selected_block.get_dont_get() == True:
                curr_state = 0
                loop_state_1 = 0
                continue
            else:
                selected_block.grab_block()
                curr_state += 1

        # CHECK COLOUR STATE
        if curr_state == 2:
            print("State 2")
            getting_pos = False
            
            # Create msg
            msg = JointState(
                # Set header with current time
                header=Header(stamp=rospy.Time.now()),
                name=['joint_1', 'joint_2', 'joint_3', 'joint_4']
            )

            # Set a middle pose to avoid collision
            theta_list_mid = [0, 0, np.deg2rad(-66), np.deg2rad(10)]
            msg.position = theta_list_mid
            show_cam_pub.publish(msg)

            # Move to show camera position
            theta_list = [0, np.deg2rad(-35), np.deg2rad(-55), np.deg2rad(-100)]
            msg.position = theta_list
            show_cam_pub.publish(msg)

            # Wait for robot to get into pose
            time.sleep(1.5)

            # Check colour
            getting_colour = True
            time.sleep(0.1)

            # Move onto next state            
            curr_state += 1 


        # DROPOFF BLOCK STATE
        if curr_state == 3 :
            print("state 3")
            # Dont get new position while dropping off block
            getting_pos = False
            
            # Drop off the block
            selected_block.dropoff_box()

            # Remove the block from the list of blocks on the carousel
            block_list.remove(selected_block)

            # Move back to state 0
            loop_state_1 = 0
            curr_state = 0

    rospy.spin()

if __name__ == "__main__":
    main()    
