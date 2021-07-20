from quanser.hardware import HIL
from quanser.multimedia import (Video3D, Video3DStreamType, ImageFormat, ImageDataType)
from quanser.common import GenericError
import cv2
import numpy as np
from array import array
import time, math

#class QBot2e
#Defines all functionality for interacting with the mobile robot base
#Includes functions for reading and writing digital, analog and other I/O on
#robot chassis
class QBot2e:

    # Define class-level variables 
    _card = None
    _capture = None
    _RGB_stream = None
    _ai_channels = None
    _ai_buffer = None
    _di_channels = None
    _di_buffer = None
    _do_channels = None
    _do_buffer = None
    _oi_channels = None
    _oi_buffer = None
    _gyro_z_bias = None
    _oo_channels = None
    _oo_buffer = None
    _enc_channels = None
    _enc_buffer = None
    _image_data = None


    _qbot_diameter = 0.235
    
    # Initilize QBot2e
    def __init__(self):
        print ("Initializing QBot2e...")
        
        # Define DAQ type
        self._card = HIL()
        self._card.open("qbot2e", "0")

        # Configure analog channels
        self._ai_channels = array('I', [0, 4, 5])
        self._ai_buffer = array('d', [0.0] * len(self._ai_channels))
        
        # Configure digital channels
        self._do_channels = array('I', [i for i in range(28,36)])
        self._do_buffer = array('b', [0, 0, 0, 0, 1, 1, 1, 1])
        self._di_channels = array('I', [i for i in range(28,59)])
        self._di_buffer = array('b', [0] * len(self._di_channels))
        #self._card.set_digital_directions(self._di_channels, self._di_num_channels, self._do_channels, self._do_num_channels)
    
        # Configure other channels 
        self._oo_channels = array('I', [2000, 2001])
        self._oo_buffer = array('d', [0.0] * len(self._oo_channels))
        self._oi_channels = array('I', [1002, 3000, 3001, 3002, 11000, 11001, 12000, 16000])
        self._oi_buffer = array('d', [0.0] * len(self._oi_channels))

        # Configure encoder channels
        self._enc_channels = array('I', [0, 1])
        self._enc_buffer = array('i', [0] * len(self._enc_channels))
        
        # Reset QBot2e
        self.reset()

        print("QBot2e Initialized")
    
    ################## ANALOG IN METHODS ##################
    
    # Get analog input buffers
    def update_ai_buffer(self): 
        self._card.read_analog(self._ai_channels, len(self._ai_channels), self._ai_buffer)
        
    # Read battery voltage
    def get_batt_volts(self):
        self.update_ai_buffer()
        return self._ai_buffer[0]
    
    ################## ENCODER METHODS ####################
    
    # Get/set encoder values
    def update_enc_buffer(self): 
        self._card.read_encoder(self._enc_channels, len(self._enc_channels), self._enc_buffer)
    def push_enc_buffer(self): 
        self._card.set_encoder_counts(self._enc_channels, len(self._enc_channels), self._enc_buffer)
        
    # Read raw encoder counts
    def read_encoder_count(self, _channel):
        self.update_enc_buffer()
        return self._enc_buffer[_channel]
    
    ################## DIGITAL I/O METHODS ################
    
    # Get/set digital I/O buffers
    def update_di_buffer(self): 
        self._card.read_digital(self._di_channels, len(self._di_channels), self._di_buffer)
    def push_do_buffer(self): 
        self._card.write_digital(self._do_channels, len(self._do_channels), self._do_buffer)
        
    # Set LED outputs
    def set_leds(self, bLED = [0, 0, 0, 0]):
        self._do_buffer[0:4] = array('b', bLED)
        self.push_do_buffer()
        
    # Read all digital in
    def read_din(self):
        self.update_di_buffer()
        return self._di_buffer
    
    # Return bump sensor inputs
    def read_bump_sensors(self):
        self.update_di_buffer()
        return self._di_buffer[0:3]
        
    # Return button inputs
    def read_buttons(self):
        self.update_di_buffer()
        return self._di_buffer[8:11]
        
    # Read right dock IR; [NR, NC, NL, FR, FC, FL]
    def read_right_dock_ir(self):
        self.update_di_buffer()
        return self._di_buffer[13:19]
        
    # Read center dock IR; [NR, NC, NL, FR, FC, FL]
    def read_center_dock_ir(self):
        self.update_di_buffer()
        return self._di_buffer[19:25]
        
    # Read left dock IR; [NR, NC, NL, FR, FC, FL]
    def read_left_dock_ir(self):
        self.update_di_buffer()
        return self._di_buffer[25:31]
    
    ################## OTHER I/O METHODS ##################
    
    # Get/set other I/O buffers
    def update_oi_buffer(self): 
        self._card.read_other(self._oi_channels, len(self._oi_channels), self._oi_buffer)
    def push_oo_buffer(self):
        self._card.write_other(self._oo_channels, len(self._oo_channels), self._oo_buffer)
    
    # Read z gyro
    def read_z_gyro(self):
        self.update_oi_buffer()
        z_gyro = self._oi_buffer[3] - self._z_bias
        return z_gyro
        
    # Measure gyro z-axis bias
    def update_gyro_z_bias(self):
        gyro = 0.0
        for i in range(10000):
            self.update_oi_buffer()
            gyro += self._oi_buffer[3]
        self._gyro_z_bias = gyro/10000.0
        print ("Z-axis gyro bias: {}".format(self._gyro_z_bias))
        
    # Set motor speed command
    def set_velocity(self, velocity = [0, 0]):
        self._oo_buffer = array('d', velocity)
        self.push_oo_buffer()

    ################## CONTROL METHODS ####################
    
    # Move a set distance and angle open loop
    def move_time(self, distance = 0, angle = 0, move_time = 1):
        # Ignore command if time <= 0 seconds
        if(move_time > 0):
            avg_linear_velocity = distance / move_time
            avg_angular_velocity = angle / move_time
            linear_velocity_delta = avg_angular_velocity * (self._qbot_diameter / 2)
            velocity_command = [avg_linear_velocity + linear_velocity_delta, avg_linear_velocity - linear_velocity_delta]
            self.set_velocity(velocity_command)
            time.sleep(move_time)
            self.halt()
    
    # Move a set distance and angle using odometry
    def move_odo(self, distance = 0, angle = 0, move_time = 1):
        # TODO
        time.sleep(move_time)
        
    # Move a set distance and angle using gyro
    def move_gyro(self, distance = 0, angle = 0, move_time = 1):
        # TODO
        time.sleep(move_time)
    
    # Stop both motors
    def halt(self):
        self.set_velocity([0, 0])
    
    ################## DEVICE MANAGEMENT ##################
    
    # Reset QBot2e
    def reset(self):
        print ("Resetting QBot2e...")
        
        #Clear LEDs
        self.set_leds([0, 0, 0, 0])

        # Stop motors
        self.set_velocity([0, 0])
        
        # Reset encoders to 0
        self._enc_buffer = array('I', [0, 0])
        self.push_enc_buffer()
        
        # Sample gyroscope z-axis bias
        if self._gyro_z_bias == None:
            self.update_gyro_z_bias()

    # Close DAQ
    def close(self):
        self.__exit__()
        
    # Exit routine
    def __exit__(self):
        print ("Closing DAQ...")
        self.reset()
        self._card.close()
        print ("QBot2e closed")

#####################################################################

#class Kinect
#Defines methods for initializing the kinect and getting images 
class Kinect:

    # Define class-level variables
    _image_width = 640
    _image_height = 480
    _image_rate = 10
    _image_buffer = None
    _stream_index = 0
    _capture = None
    _RGB_stream = None
    _status = 0

    # Initialize Kinect
    def __init__(self, ID = "0", rate = 10):
        print ("Initializing Kinect")
        if rate != 10:
            self._image_rate = rate
        self._image_buffer = self.placeholder_image()
        rcv_frame = False
        fail_count = 0
        while rcv_frame == False:
            if self._RGB_stream != None:
                self.halt()
            try:
                self._capture = Video3D(ID)
                print("Created Video3D instance")
                self._RGB_stream = self._capture.stream_open(Video3DStreamType.COLOR, self._stream_index,
                    self._image_rate, self._image_width, self._image_height,
                    ImageFormat.ROW_MAJOR_INTERLEAVED_BGR, ImageDataType.UINT8)
                print("Opened RGB stream")
                self._capture.start_streaming()
            except GenericError as ex:
                print("ERROR: " + ex.get_error_message())

            print("Waiting for Kinect...")
            t_start = time.monotonic()
            while (time.monotonic() - t_start) < 5:
                frame = self._RGB_stream.get_frame()
                if frame != None:
                    rcv_frame = True
                    self._status = 1
                    break
            fail_count += 1
            if fail_count > 2:
                print("too many tries, aborting")
                break
            
        print ("Kinect Initialized")

    #Get an RGB frame from the Kinect
    def get_RGB_frame(self):
        frame = self._RGB_stream.get_frame()
        if frame != None:
            frame.get_data(self._image_buffer)
            frame.release()
        return self._image_buffer

    #Return the status of the Kinect (are frames being received)
    def get_status(self):
        return self._status

    #Return a placeholder image
    def placeholder_image(self):
        return cv2.imread('robot.jpg')

    #Cleanup for shutting down the Kinect
    def halt(self):
        print("Stopping stream")
        self._capture.stop_streaming()
        self._capture = None
        print("Closing stream")
        self._RGB_stream.close()
        self._RGB_stream = None
        print("Kinect halted")
        self._status = 0

