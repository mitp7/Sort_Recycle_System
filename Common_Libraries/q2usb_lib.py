from quanser.hardware import HIL
from array import array
import time

class q2usb:

    # Define class-level variables 

    # Q2-USB variables
    _card = None
    _ai_channels = None
    _ai_buffer = None
    _ao_channels = None
    _ao_buffer = None
    _enc_channels = None
    _enc_buffer = None
       
    # Initilize Q2-USB
    def __init__(self):
        # Define DAQ type as Q2-USB
        self._card = HIL()
        self._card.open("q2_usb", "0")

        # Set Q2-USB speed to noraml
        self._card.set_card_specific_options("update_rate=normal;",50)

        # Configure analog input channels
        self._ai_channels = array('I', [0, 1])
        self._ai_buffer = array('d', [0.0] * len(self._ai_channels))

        # Configure analog output channels
        self._ao_channels = array('I', [0, 1])
        self._ao_buffer = array('d', [0.0] * len(self._ao_channels))

        # Configure encoder input channels
        self._enc_channels = array('I', [0, 1])
        self._enc_buffer = array('i', [0] * len(self._enc_channels))

        # Write 0 V to all AO channels
        self._card.write_analog(self._ao_channels, len(self._ao_channels), self._ao_buffer)

        # Set encoder counts to 0
        _init_enc_counts = array('i', [0, 0])
        self._card.set_encoder_counts(self._enc_channels, len(self._enc_channels), _init_enc_counts)

        print ("Q2-USB DAQ Initialized")

    ######################### Q2-USB ANALOG IN METHODS ######################### 

    # Read single analog input channel
    def read_analog_input (self, channel):
        self._card.read_analog(self._ai_channels, len(self._ai_channels), self._ai_buffer)
        return self._ai_buffer[channel]
    
    ######################### Q2-USB ANALOG OUT METHODS ######################### 
    
    # Write value to single analog output channel
    def write_analog_output (self, channel, value): 
        self._card.write_analog(array('I', [channel]), 1, array('d', [value]))
        
    ######################### Q2-USB ENCODER METHODS ######################### 
       
    # Read raw encoder counts for single channel
    def read_encoder_count(self, channel):
        self._card.read_encoder(self._enc_channels, len(self._enc_channels), self._enc_buffer)
        return self._enc_buffer[channel]

    ################## DEVICE MANAGEMENT ##################
    
    # Close Q2-USB
    def close(self):
        self.__exit__()
    
    # Exit routine
    def __exit__(self):
        print ("Closing Q2-USB ...")
        # Write 0 V AO channels 0 and 1
        self.write_analog_output (0, 0) 
        self.write_analog_output (1, 0)
                
        # Close DAQ
        self._card.close()
        
        print ("Q2-USB closed")
