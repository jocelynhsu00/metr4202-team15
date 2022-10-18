import Logic as L

state = 0
while(1):

    if state == 0 :
        #initialise 
        
        #find blocks (not sure how to do this should be in max's code)

        #create block classes 
        # blue = L.Block(x, y, z)
        # green = L.Block(x, y, z)
        # red = L.Block(x, y, z)
        # yellow = L.Block(x, y, z)

        state += 1 

                

    if state == 1 :
        #choose what block to pick up 
        #Not sure how to do this

        # selected_block = green # or red or blue or what ever you chose 
        L.grip_open()
        state += 1 


    if state == 2 :
        #Grab and move 
        # selected_block.L.grab_box()
        L.grip_box()
        state += 1 



    if state == 3 :
        #reset 
        L.grip_close()
        state = 0
        
