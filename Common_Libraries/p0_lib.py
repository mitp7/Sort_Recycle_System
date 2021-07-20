# This code represents the library that students will reference to complete activities in for Project 3
#
# All code is currently set up to run on a raspberry pi ONLY.

import math
import sys
import time
sys.path.append('../')

from Common_Libraries.quanser_sim_lib import QBot2e_sim, CameraUI
from Common_Libraries.postman import postman
from Common_Libraries.modular_comm import comm_modular_container

QBOT_DIAMETER = 0.235
camera_bumper_depth = 0.156
row = 360 # centre pixel is 240 , bottom pixel is 480
col = 320
#Initialize comms
QIL = postman(18001)
comc = comm_modular_container()


class qbot:
    def __init__(self, speed):
        self.bot = QBot2e_sim(QIL, 0)#Initialize qbot sim
        self.maxSpeed = 100
        self.speed = speed
        self.turn = 0
        #img_RGB = self.bot.get_new_RGB() #Get RGB image buffer (and request new image frame)

    def forward(self, time):
        #Convert speed and turn to wheel speeds and command QBot
        delta = self.turn * QBOT_DIAMETER
        left = self.speed - delta
        right = self.speed + delta
        velocity = [left,right] # velocity for the left and right wheel
        self.bot.move_time(velocity,time)
    
    def travel_forward(self, threshold):
        # Initial depth measurement in meters
        d = self.bot.measure_depth(row, col) - camera_bumper_depth
        print ("Depth (m): ", d)
        # Drive 
        self.bot.set_velocity([self.speed,self.speed])
        
        # Continue to drive until threshold is reached
        while threshold < d:
            print ("Depth (m): ", d)
            d = self.bot.measure_depth(row, col) - camera_bumper_depth
            time.sleep(.3)
        
        # Stop QBot
        self.bot.halt()

    def rotate(self, degree):
        time = 1
        rad = math.radians(degree)
        speed = ((QBOT_DIAMETER / 2.0) * rad / time)
        velocity = [speed, -speed]
        self.bot.move_time(velocity,time)

    # Read and return how far the Q-Bot is from an object e.g. the walls.    
    def depth(self):
        return self.bot.measure_depth(row, col) - camera_bumper_depth

   # Used to establish continuous communication 
    def ping(self):
        self.bot.ping()
