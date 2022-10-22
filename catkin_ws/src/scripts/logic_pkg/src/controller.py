#!/usr/bin/env python3

from logic_pkg.src.logic_task_1 import *
from time import *

curr_state = 0
while(1):

    if curr_state == 0 :
        #initialise 


        print('State 0')
        #print(camera_list())
        # publish()
        curr_state += 1 

                

    if curr_state == 1 :
        #choose what block to pick up 
        #Not sure how to do this

        # selected_block = green # or red or blue or what ever you chose 
        print('State 1')
        #grip_open()
        sleep(1)
        curr_state += 1


    if curr_state == 2 :
        #Grab and move 
        # selected_block.L.grab_box()
        print('Start 2')
        #grip_box()
        sleep(1)
        curr_state += 1 



    if curr_state == 3 :
        #reset 
        print('start 3')
        #grip_close()
        sleep(1)

        curr_state = 0
