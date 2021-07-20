import time
import random
import sys
sys.path.append('../')

from Common_Libraries.p3b_lib import *

import os
from Common_Libraries.repeating_timer_lib import repeating_timer

def update_sim():
    try:
        my_table.ping()
    except Exception as error_update_sim:
        print (error_update_sim)

### Constants
speed = 0.2 #Qbot's speed

### Initialize the QuanserSim Environment
my_table = servo_table()
arm = qarm()
arm.home()
bot = qbot(speed)

##---------------------------------------------------------------------------------------
## STUDENT CODE BEGINS
##---------------------------------------------------------------------------------------

"""
Tues - 35
Mit Patel (patem97)
Alfred Zhou (zhoua29)
"""

"""
Function: loadContainer()
Purpose: dispense, and load container(s) on the Q-Bot if they go to the same target bin
Input:
- deposit_counter, tracks how many times the Q-Bot deposits the container(s)
- next_container, bin ID of the container which is not loaded on the Q-Bot
Output: a list of the target locations and a boolean operator indicating if 3 containers are loaded
"""

def loadContainer(deposit_counter, next_container):
    
    while (True):
        #Tracks the total mass of the containers
        total_mass = 0
        
        #Dispenses and loads the first container
        while (True):
            #Stores the information of the dispensed container (material, bin destination)
            container_list = []
            
            #Stores the bin ID of the first loaded container
            first_loaded_container = []

            #Checks if this is the first or second time a container is loaded to transfer 
            if (deposit_counter >= 1):
                
                #Stores the bin ID of the container which is not loaded
                first_target_bin = next_container[len(next_container)-1]
            else:
                
                #Dispenses a random container as this is the first time
                container_properties = my_table.container_properties(random.randint(1,6))
                container_list.append(container_properties)
                my_table.dispense_container()

                #Stores the bin ID from the container properties (material,mass,bin destination)
                for properties in container_list:
                    total_mass = properties[1]
                    first_target_bin = properties[2]

            print("First Target Bin:  ", first_target_bin)

            #Checks if dispensed container matches bin ID
            if (first_target_bin == "Bin01"):
                bin_ID = "Bin01"
            elif (first_target_bin == "Bin02"):
                bin_ID = "Bin02"
            elif (first_target_bin == "Bin03"):
                bin_ID = "Bin03"
            elif (first_target_bin == "Bin04"):
                bin_ID = "Bin04"
            
            #Loads the first container on the Q-Bot
            if (first_target_bin == bin_ID and total_mass < 90):
                #Appends the containers bin ID to compare with second dispensed container
                first_loaded_container.append(bin_ID) 
                arm.move_arm(0.68, 0.0, 0.2496)
                arm.control_gripper(45)
                arm.move_arm(0.2256, 0.0, 0.1898)
                arm.move_arm(-0.11, -0.300, 0.6)
                arm.move_arm(-0.11, -0.44, 0.39)
                arm.control_gripper(-45)
                arm.rotate_elbow(-33)
                arm.home()
                break
            else:
                break

        #Dipenses and loads the second container (if same bin ID as first container) 
        while (True):
            #Stores the information of the dispensed container
            container_list = []
            #Stores the bin ID of the second loaded container
            second_loaded_container = []

            #Dispenses a random container
            container_properties = my_table.container_properties(random.randint(1,6))
            container_list.append(container_properties)
            my_table.dispense_container()

            #Stores the bin ID from the container properties (material,mass, bin destination)
            for properties in container_list:
                total_mass += properties[1]
                second_target_bin = properties[2]
                
            print("Second Target Bin: ", second_target_bin)
            
            #Only loads a container if the mass is less than 90
            if(total_mass < 90):
                #Checks if the dispensed container matches bin ID
                if (second_target_bin == "Bin01"):
                    bin_ID = "Bin01"
                elif (second_target_bin == "Bin02"):
                    bin_ID = "Bin02"
                elif (second_target_bin == "Bin03"):
                    bin_ID = "Bin03"
                elif (second_target_bin == "Bin04"):
                    bin_ID = "Bin04"

                #Checks if the bin ID of second container matches the first container dispensed
                if (second_target_bin == bin_ID and first_loaded_container[0] == bin_ID):
                    #Appends the containers bin ID to compare with third dispensed container
                    second_loaded_container.append(bin_ID)
                    arm.move_arm(0.68, 0.0, 0.2496)
                    arm.control_gripper(45)
                    arm.move_arm(0.2256, 0.0, 0.1898)
                    arm.move_arm(0, -0.300, 0.6)
                    arm.move_arm(0.0, -0.50, 0.39)
                    arm.control_gripper(-23)
                    arm.rotate_elbow(-38)
                    arm.home()
                    break
                else:
                    #Does not load a container if the total mass is greater than 90
                    break
            else:
                break
            
        #Dipenses and loads the third container (if same bin ID as second container) 
        while(len(second_loaded_container) > 0):
            #Tracks if a third container is picked up inorder to dispense another container
            third_deposit = False
            
            #Stores the information of the dispensed container (material, bin destination)
            container_list = []

            #Dispenses a random container
            container_properties = my_table.container_properties(random.randint(1,6))
            container_list.append(container_properties)
            my_table.dispense_container()

            #Stores the bin ID from the container properties  (material,mass,bin destination)
            for properties in container_list:
                total_mass += properties [1]
                third_target_bin = properties[2]

            
            print("Third Target Bin:  ", third_target_bin)

            #Only loads a container if the total mass is less than 90
            if(total_mass < 90):
                
                #Checks if the dispensed container matches bin ID
                if (third_target_bin == "Bin01"):
                    bin_ID = "Bin01"
                elif (third_target_bin  == "Bin02"):
                    bin_ID = "Bin02"
                elif (third_target_bin  == "Bin03"):
                    bin_ID = "Bin03"
                elif (third_target_bin == "Bin04"):
                    bin_ID = "Bin04"
                    
                #Checks if the bin ID of third container matches the second container dispensed
                if (third_target_bin == bin_ID and second_loaded_container[0] == bin_ID and total_mass < 90):
                    arm.move_arm(0.68, 0.0, 0.2496)
                    arm.control_gripper(45)
                    arm.move_arm(0.2256, 0.0, 0.1898)
                    arm.move_arm(0.12, -0.300, 0.6)
                    arm.move_arm(0.12, -0.44, 0.39)
                    arm.control_gripper(-23)
                    arm.rotate_elbow(-38)
                    arm.home()
                    #Changes to True indicating 3 containers are loaded
                    third_deposit = True
                    break
                else:
                    #Does not load a container if the total mass is greater than 90
                    break
            else:
                break

        print("Target Location:   ", first_target_bin)


        #Tracks the containers which are placed on the sorting table
        #Used for the infinite loading and depositing process
        target_location = []

        #Appends the bin ID's of the first and second container dispensed
        target_location.append(first_target_bin)
        target_location.append(second_target_bin)

        #Appends the bin ID of the third container only if a second container is loaded
        if(len(second_loaded_container) > 0):
            target_location.append(third_target_bin)
            return target_location, third_deposit
        else:
            third_deposit = False
            return target_location, third_deposit

        return target_location, third_deposit
        break


"""
Function: transferContainer()
Purpose: transfers the containers to the target bins
Input:
- transfer_location, bin ID of the first container loaded on the Q-Bot
Output: no output
"""

def transferContainer(transfer_location):
    bot.activate_ultrasonic_sensor()

    #Checks if the loaded container matches bin ID and assigns a sensor value
    if (transfer_location == "Bin01"):
        bin_ID = "Bin01"
        #Value collected from ultrasonic reading which indicates when the Q-Bot should stop for each bin
        bin_target_location = [0.1]
        
    elif (transfer_location == "Bin02"):
        bin_ID = "Bin02"
        bin_target_location = [0.15]
        
    elif (transfer_location == "Bin03"):
        bin_ID = "Bin03"
        bin_target_location = [0.20]
        
    elif (transfer_location == "Bin04"):
        bin_ID = "Bin04"
        bin_target_location = [0.24, 0.25]

    if(transfer_location  == bin_ID):
        #While loop which runs until the target bin is located using ultrasonic sensor
        #Follows the yellow line until it arrives at target bin
        lines = 0
        while(lines < 2):
            lines, velocity = bot.follow_line(0.07)
            bot.forward_velocity(velocity)
            ultrasonic_reading = bot.read_ultrasonic_sensor(bin_ID)

            #Checks if assigned bin locations match the sensor values to stop the Q-Bot
            if(ultrasonic_reading in bin_target_location):
                
                #Stops the Q-Bot parallel to the bin
                bot.stop()
                bot.deactivate_ultrasonic_sensor()
                print("Reached target bin...")
                time.sleep(1)
                print("Deposit Container...")
                break
            else:
                #Moves Q-Bot forward until target location is determined
                bot.forward_speed(0.06)

"""
**Bonus Part of the Project**

Function: processFile() 
Purpose: extracts the angles and timings from the modelling simulation data
Input:
- data_file, file containing times and angles to rotate the hopper
Output: two lists, one containing time and another with angle data for the hopper
"""

def processFile(data_file):
    #List to store the extracted angles
    hopper_angles = []
    #List to store the extracted times
    hopper_times = []
    
    my_file = open(data_file, "r")

    for line in my_file:
        #Split up the timings and the angles
        data = line.split()

        #Append the data to the coressponding lists created above
        hopper_times.append(float(data[0]))
        hopper_angles.append(float(data[1]))
        
    return hopper_times, hopper_angles

"""
Function: depositContainer()
Purpose: deposits the container(s) into the target bins
Input:
- deposit_location, bin ID of the first container loaded on the Q-Bot
- rotation_times, list containing timings extracted from the simulation/txt file
- rotation_angles, list containing angles extracted from the simulation/txt file
Output: no output
"""

def depositContainer(deposit_location, rotation_times, rotation_angles):
    #Checks if the loaded container matches bin ID and assigns appropiate depositing values
    if(deposit_location == "Bin01"):
        bin_ID = "Bin01"
        #Time to move the Q-Bot towards the bin
        forward_bin = 0.28
        #Time to move the Q-Bot towards the yellow line after depositing
        towards_yellow = 0.28
        
    elif(deposit_location == "Bin02"):
        bin_ID = "Bin02"
        forward_bin = 0.7 
        towards_yellow = 0.8
        
    elif(deposit_location == "Bin03"):
        bin_ID = "Bin03"
        forward_bin = 0.9
        towards_yellow = 1.1
        
    elif(deposit_location == "Bin04"):
        bin_ID = "Bin04"
        forward_bin = 1.2
        towards_yellow = 1.2

    #Checks if assigned bin locations matches the bin ID to deposit the container in the bin
    if(deposit_location == bin_ID):
        
        #Rotates clockwise facing the bin
        bot.rotate(90)
        time.sleep(1)
        #Moves towards the bin based on the forward_bin values assigned for each one
        bot.forward_time(forward_bin) 
        time.sleep(1)
        #Rotates counter-clockwise inorder to dump the container
        bot.rotate(-91)

        #Activates the actuator
        bot.activate_actuator()

        #Iterates in the angle and time list 
        for i in range(len(rotation_angles)):
            #Rotates the hopper using the positive angle data
            bot.rotate_actuator(-1*rotation_angles[i])

            #Checks if the end of the list is reached to deactivate the actuator
            if (i == (len(rotation_angles)-1)):
                bot.deactivate_actuator()
                break
            else:
                #Uses the times from the file to match with the rotation
                times = rotation_times[i+1] - rotation_times[i]
                time.sleep(times)

        print("Return to home position...")

        #Rotates counter-clockwise and moves towards the yellow line
        bot.rotate(-90) 
        time.sleep(1)
        bot.forward_time(towards_yellow) 
        time.sleep(1)
        #Turn right facing yellow line
        bot.rotate(90) 


"""
Function: returnHome()
Purpose: returns the Q-Bot home after depositing the container(s)
Input: no input
Output: no output
"""

def returnHome():
    lines = 0
    while(lines < 2):
        lines, velocity = bot.follow_line(0.08)
        bot.forward_velocity(velocity)
    bot.forward_time(0.35)



"""
Continues the loading and depositing process infinetly until user stops 
"""

#Tracks how many times a container has been deposited
loop_counter = 0

#Stores the bin ID of the container which is not loaded (on sorting table)
unloaded_container = []

#Loop iterates infinetly until stopped by the user
while(True):

    #Stores the bin ID's of all of the containers either loaded or on sorting table
    final_location, third_container = loadContainer(loop_counter, unloaded_container)

    #Checks if the Q-Bot returned home and rotates after it has been loaded with containers
    if (loop_counter >= 1):
        bot.rotate(185)

    #Passing the target location of the container to transfer and deposit the container
    transferContainer(final_location[0])

    #Passess the final location, hopper rotation and timings into the deposit function
    hopper_times, hopper_angles = processFile("data.txt")
    depositContainer(final_location[0], hopper_times, hopper_angles)

    #Return the Q-Bot home after depositing
    returnHome()
    print("_____________________________________________________________")

    #If three containers are loaded a new container is dispensed and deposited
    if (third_container == True):
        final_location, third_container = loadContainer(loop_counter, unloaded_container)

        transferContainer(final_location[0])
        hopper_times, hopper_angles = processFile("data.txt")
        depositContainer(final_location[0], hopper_times, hopper_angles)
        returnHome()

        #Checks if three containers are deposited or less
        #If less than it will load the container from the table to the Q-Bot
        if (len(final_location) < 3):
            #Increments the loop count, indicates that Q-Bot is at home position
            loop_counter += 1
            #Loads the container on the sorting table
            unloaded_container.append(final_location[len(final_location)-1])
    else:
        loop_counter += 1
        unloaded_container.append(final_location[len(final_location)-1])
 

##---------------------------------------------------------------------------------------
## STUDENT CODE ENDS
##---------------------------------------------------------------------------------------
update_thread = repeating_timer(2,update_sim)
