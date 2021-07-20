"""
This code represents the library that students will reference to complete LAB activities for Project 3
All code is currently set up to run on a raspberry pi ONLY.

Libaries were split into two due to the backend operation of the simulation. P3a_lib.py shall be used in the lab and
P3b_lib.py in the design studio

Items to mention to students
1. Rotation limits base +/- 175 deg, shoulder +/- 90 deg, elbow +90 -80 deg, wrist +/-170, gripper 0(open)-1(close)
2. P3a_lib.py shall be used in the lab and P3b_lib.py in the design studio
"""

# This code represents the library that students will reference to complete activities in for Project 3
#
# All code is currently set up to run on a raspberry pi ONLY.

# Items to mention to students
# 1. Rotation limits base +/- 175 deg, shoulder +/- 90 deg, elbow +90 -80 deg, wrist +/-170, gripper 0(open)-1(close)

# Import all required libraries
import sys
sys.path.append('../')
import numpy as np
import time
import os
import math

from Common_Libraries.postman import postman
from Common_Libraries.modular_comm import comm_modular_container
from Common_Libraries.quanser_sim_lib import rotarytable_sim, QArm_sim, QBot2e_sim, CameraUI

from Common_Libraries.modular_comm import comm_modular_server
from array import *

import random

#initialize the QuanserSim environment

QIL = postman(18001) #Establish communication
loop_counter = 0
servo_speed = 0.15
interval = 0.2

# Servo Table Constants
# Container properties
empty_plastic_container = 9.25 # empty mass of plastic container in g
empty_metal_container = 15.0 # empty mass of metal container in g
empty_paper_container = 10.0 # empty mass of paper container in g

# QBot Constants
QBOT_DIAMETER = 0.235
camera_bumper_depth = 0.156
row = 360 # centre pixel is 240 , bottom pixel is 480
col = 320

class servo_table:
    def __init__(self):
        self.my_table = rotarytable_sim(QIL)
        self.table_weight = 0
        self.proximity = False
        
    def rotate_table_speed(self, speed):
        if float(speed) >= 0.0 and float(speed) <= 2.0:
            print("I am here")
            self.my_table.rotate_clockwise(speed)
        elif float(speed) > 2.0:
            print("Input speed is too fast. Enter a speed less than 2 m/s")
        elif float(speed) < 0.0:
            print("Input a positive speed.")
        else:
            print("Invalid input.")

    def rotate_table_angle(self, deg):
        if deg < 0:
            print("Input a positive angle.")
        else:
            self.my_table.command_rel_position_pid(deg)

    def stop_table(self):
        self.my_table.stop_table()

 # -----------------------------------------------------------------------------------------------
 # Not necessarily useful to students. Leave it out of the documentation
    def ping(self):
        self.my_table.ping()

# -----------------------------------------------------------------------------------------------
    # Not necessarily useful to students. Leave it out of the documentation
    # dispense bottle based on a random number between 0 and 5 inclusive. Where 1 = non contaminated plastic
    # 2 = non-contaminated metal, 3 = non-contaminated paper, 4 = contaminated plastic, 5 = contaminated metal
    # 6 = contaminated paper
    def dispense_container(self,rand_number):
        if int(rand_number) >= 1 and int(rand_number) <= 6:
            self.obj_number = rand_number - 1

            # non-contaminated containers - a list containing the color of the object and the mass in grams
            obj1 = ['clear','plastic',empty_plastic_container] # UPDATE TO REMOVE THE LAST LIST ITEM???
            obj2 = ['red','metal',empty_metal_container]
            obj3 = ['blue','paper',empty_paper_container]

            # contaminated containers - a list containing the color of the object and the mass in grams
            obj4 = ['clear','plastic',empty_plastic_container + random.uniform(5.0, 50.0)]
            obj5 = ['red','metal',empty_metal_container + random.uniform(5.0, 50.0)]
            obj6 = ['blue','paper',empty_paper_container + random.uniform(5.0, 50.0)]

            self.container = [obj1,obj2,obj3,obj4,obj5,obj6]

            self.container_color = self.container[self.obj_number][0]
            self.material = self.container[self.obj_number][1]
            self.container_mass = round(self.container[self.obj_number][2],2)

            if self.container_color == "red":
                color = [1,0,0]
            elif self.container_color == "clear":
                color = [0.86,0.94,0.94]
            else:
                color = [0,0,1]

            self.my_table.spawn_single_bottle(color,self.container_mass,self.material)
            time.sleep(1)
        else:
            print("Enter a number between 1 and 6 (inclusive).")

    # Outputs the mass readings from the table load cell given a duration input.
    # A single mass reading is appended to a list at given interval steps (0.2) for the entire duration
    def load_cell_sensor(self, duration):
        load_cell_mass = round(self.my_table.read_load_cell(),2)
        mass = []    
        elapsed_time = 0
        start_time = time.time()
        while elapsed_time < duration:
            if load_cell_mass > 0.0:
                mass.append(load_cell_mass + random.uniform(0.0, 0.4))
            else:
                mass.append(load_cell_mass)
            time.sleep(interval)
            elapsed_time = time.time() - start_time
        if self.tof_sensor()<=2 or self.tof_sensor()<=13:
            return mass

    # returns the position of the bottle from the short tower                                     
    def proximity_sensor_short(self):
        distance = self.my_table.read_proximity_sensor_short()
        if distance[0] != 0 or distance[1] != 0 or distance[2] != 0:
            self.proximity_short = True
        else:
            self.proximity_short = False
        return self.proximity_short

    # returns the position of the bottle from the tall tower                                     
    def proximity_sensor_tall(self):
        distance = self.my_table.read_proximity_sensor_tall()
        if distance[0] != 0 or distance[1] != 0 or distance[2] != 0:
            self.proximity_tall = True
        else:
            self.proximity_tall = False
        return self.proximity_tall

    # returns the height of the gap between the top of the tall tower and the top of the container
    def tof_sensor(self):
        distance = self.my_table.read_tof_sensor()
        return distance

    # Gives a high reading if an object is within it's proximity
    def capacitive_sensor(self):
        self.proximity_sensor_short()
        if self.proximity_short == True:
            reading = random.uniform(4.5, 5.0)
        else:
            reading = random.uniform(0, 0.4)
        return reading

    # Gives a high reading metal is detected.
    # To randomize the values, a duration is input and readings are appended to a table for a given intervals (0.2)
    # during that specified time.
    def inductive_sensor(self, duration):
        if self.material == 'metal':
            reading = []
            elapsed_time = 0
            start_time = time.time()
            while elapsed_time < duration:
                reading.append(random.uniform(4.5, 5.0))
                time.sleep(interval)
                elapsed_time = time.time() - start_time
        else:
            reading = []
            elapsed_time = 0
            start_time = time.time()
            while elapsed_time < duration:
                reading.append(random.uniform(0, 0.4))
                time.sleep(interval)
                elapsed_time = time.time() - start_time
        if self.tof_sensor() <= 15:
            return reading

    # Dark On Retro-reflective photoelectric sensor. It read a high value when it senses a target and a low value
    # when the light comes back to the receiver (e.g no target / clear target)
    # To randomize the values, a duration is input and readings are appended to a table for a given intervals (0.2)
    # during that specified time.
    def photoelectric_sensor(self, duration):
        if self.material == "paper" or self.material == "metal":
            reading = []
            elapsed_time = 0
            start_time = time.time()
            while elapsed_time < duration:
                reading.append(random.uniform(4.5, 5.0))
                time.sleep(interval)
                elapsed_time = time.time() - start_time
        else:
            reading = []
            elapsed_time = 0
            start_time = time.time()
            while elapsed_time < duration:
                reading.append(random.uniform(0, 0.4))
                time.sleep(interval)
                elapsed_time = time.time() - start_time
        if self.tof_sensor() <= 15:
            return reading
