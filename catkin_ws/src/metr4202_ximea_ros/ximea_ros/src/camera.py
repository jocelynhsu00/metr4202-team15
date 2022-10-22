#!/usr/bin/env python3
"""
Camera
"""


#always need this
import rospy

import numpy as np

# Import message types
from fiducial_msgs.msg import Fiducial, FiducialTransform, FiducialTransformArray
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
import tf2_ros
import tf



#aruco tag get position from top of the block
#bring block up to the camera by rotating the gripper(more accurate)
#leave the colour detect code as in the centre will be easier to work with as is a known position.


#get the location of the tag, publish the location, get the cube, move the cube, get colour then place
#^ will need to check if its there once moved.
#whether grab from top or bottom, will still be able to rotate, possibly easier from top
#crtl F white, use the auto enable to get a more accurate color for the block
#make it black and white while doing the aruco tags so that the fps is better and more accurate
#once confirmed pickup then change to the auto white and rgb.

#get the position and output it as something, x,y,z and theta. Will be two thetas.
#dont need to put the position into the demo camera. 

#Make a list, append the list and remove id from list when completed (goes through each block)


def transform_callback(data) -> String:
    #full_info = list(data.transforms)
    #print(data.transforms)
    
    
    
    
    for m in data.transforms:
        '''
        list format[id, transx, transy, transz, rotx, roty, rotz, rotw]
        '''
        
        id = m.fiducial_id
        print(id)
        trans = m.transform.translation
        rot = m.transform.rotation
        trans.x = trans.x + .02404*np.sin(np.pi/4+rot.z)
        trans.y = trans.y + .02404*np.cos(np.pi/4+rot.z)
        #Cameralist = [id, trans.x, trans.y, trans.z, rot.x, rot.y, rot.z, rot.w]
        Cameralist = str(f"{id},{trans.x},{trans.y},{trans.z},{rot.x},{rot.y},{rot.z},{rot.w}")

        pub.publish(Cameralist)
    #return Cameralist

        # t = TransformStamped()
        # t.child_frame_id = "fid%d" % id
        # t.header.frame_id = data.header.frame_id
        # t.header.stamp = data.header.stamp
        # t.transform.translation.x = trans.x
        # t.transform.translation.y = trans.y
        # t.transform.translation.z = trans.z
        # t.transform.rotation.x = rot.x
        # t.transform.rotation.y = rot.y
        # t.transform.rotation.z = rot.z
        # t.transform.rotation.w = rot.w
        # tf2_ros.TransformBroadcaster().sendTransform(t)



    #print(id)
    #print(trans.x)
    #print(rot)
    #print(full_info[0])

# def get_values(data):
#     x = data.transforms[0]

"""
class Aruco:
    def __init__(self):
        '''
        initialises the class
        '''
        self._id = x
        pos = 
        pass


    def get_id(self):
        '''
        returns the aruco tag id for the block
        '''
        return self._id
        pass

    def get_position(self):
        '''
        returns the position of the block
        '''
        pass


    def get_color(self):
        '''
        returns the color of the block
        '''
        pass


"""






def main():
    
    global pub
    # Create publisher
    pub = rospy.Publisher('camera_list', String, queue_size=10)
    

    # Initialise node with any node name
    rospy.init_node('listener', anonymous = True)
    rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, transform_callback)



    # You spin me right round baby, right round...
    # Just stops Python from exiting and executes callbacks
    rospy.spin()

if __name__ == '__main__':
    main()


#get aruco id
#get aruco position
#get aruco color

#green = arucotag(id = 22)
