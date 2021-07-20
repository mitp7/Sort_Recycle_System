from quanser.communications import Stream, StreamError, PollFlag, Timeout
from quanser.common import GenericError

import struct
import math
import numpy as np



######################### MODULAR CONTAINER CLASS ######################### 

class comm_modular_container:

    # Define class-level variables   
    container_size = 0
    device_id = 0       # What device type is this?
    device_number = 0   # Increment if there are more than one of the same device ID
    device_function = 0 # Command/reponse
    payload = bytearray()
    
    
    
    ID_QARM = 10
    FCN_QARM_COMMAND_AND_REQUEST_STATE = 10
    FCN_QARM_RESPONSE_STATE = 11
    FCN_QARM_COMMAND_BASE = 14
    FCN_QARM_RESPONSE_BASE = 15
    FCN_QARM_COMMAND_SHOULDER = 16
    FCN_QARM_RESPONSE_SHOULDER = 17
    FCN_QARM_COMMAND_ELBOW = 18
    FCN_QARM_RESPONSE_ELBOW = 19
    FCN_QARM_COMMAND_WRIST = 20
    FCN_QARM_RESPONSE_WRIST = 21
    FCN_QARM_COMMAND_GRIPPER = 22
    FCN_QARM_RESPONSE_GRIPPER = 23
    FCN_QARM_COMMAND_BASE_COLOR = 24
    FCN_QARM_RESPONSE_BASE_COLOR_ACK = 25
    FCN_QARM_COMMAND_ARM_BRIGHTNESS = 26
    FCN_QARM_RESPONSE_ARM_BRIGHTNESS_ACK = 27
    FCN_QARM_REQUEST_GRIPPER_OBJECT_PROPERTIES = 50
    FCN_QARM_RESPONSE_GRIPPER_OBJECT_PROPERTIES = 51
    FCN_QARM_REQUEST_END_EFFECTOR_COLLISION_SPHERE = 60
    FCN_QARM_RESPONSE_END_EFFECTOR_COLLISION_SPHERE = 61
    FCN_QARM_REQUEST_SEGMENT_COLLISIONS = 70
    FCN_QARM_RESPONSE_SEGMENT_COLLISIONS = 71
    FCN_QARM_REQUEST_RGB = 100
    FCN_QARM_RESPONSE_RGB = 101
    FCN_QARM_REQUEST_DEPTH = 110
    FCN_QARM_RESPONSE_DEPTH = 111
    
    ID_QBOT = 20
    FCN_QBOT_COMMAND_AND_REQUEST_STATE = 10
    FCN_QBOT_RESPONSE_STATE = 11
    FCN_QBOT_REQUEST_RGB = 100
    FCN_QBOT_RESPONSE_RGB = 101
    FCN_QBOT_REQUEST_DEPTH = 110
    FCN_QBOT_RESPONSE_DEPTH = 111
    
    ID_COUPLED_TANK = 30
    ID_SRV02_BASE = 40
    ID_SRV02_FLEX_LINK = 41
    ID_SRV02_BALL_AND_BEAM = 42
    ID_SRV02_PENDULUM = 42
    ID_QUBE2_SERVO = 60
    
    ID_EMG_INTERFACE = 70
    FCN_EMG_REQUEST_STATE = 10
    FCN_EMG_RESPONSE_STATE = 11
    
    
    ID_DELIVERY_TUBE = 80
    ID_AERO = 90
    
    ID_SRV02BOTTLETABLE = 100
    FCN_SRV02BT_COMMAND_SPEED = 11
    FCN_SRV02BT_REQUEST_ENCODER = 13
    FCN_SRV02BT_RESPONSE_ENCODER = 14
    FCN_SRV02BT_REQUEST_TOF = 15
    FCN_SRV02BT_RESPONSE_TOF = 16
    FCN_SRV02BT_REQUEST_PROXIMITY_SHORT = 17
    FCN_SRV02BT_RESPONSE_PROXIMITY_SHORT = 18
    FCN_SRV02BT_REQUEST_PROXIMITY_TALL = 19
    FCN_SRV02BT_RESPONSE_PROXIMITY_TALL = 20
    FCN_SRV02BT_SPAWN_CONTAINER = 21
    FCN_SRV02BT_REQUEST_LOAD_MASS = 91
    FCN_SRV02BT_RESPONSE_LOAD_MASS = 92     
    
    ID_QBOT_BOX = 110
    FCN_QBOT_BOX_COMMAND = 11    
    FCN_QBOT_BOX_COMMAND_ACK = 12
    
    ID_SCALE = 120
    FCN_SCALE_REQUEST_LOAD_MASS = 91
    FCN_SCALE_RESPONSE_LOAD_MASS = 92
    
    ID_GENERIC_SPAWNER = 130
    FCN_GENERIC_SPAWNER_SPAWN = 10
    FCN_GENERIC_SPAWNER_SPAWN_ACK = 11
    FCN_GENERIC_SPAWNER_SPAWN_WITH_PROPERTIES = 20
    FCN_GENERIC_SPAWNER_SPAWN_WITH_PROPERTIES_ACK = 21    
    
    ID_AUTOCLAVE = 140
    FCN_AUTOCLAVE_OPEN_DRAWER = 10
    FCN_AUTOCLAVE_OPEN_DRAWER_ACK = 11
    
    ID_SMARTBOX = 150
    FCN_SMARTBOX_REQUEST_SURFACE_PROPERTIES = 5
    FCN_SMARTBOX_RESPONSE_SURFACE_PROPERTIES = 6
    
    ID_UE4_SYSTEM = 1000
    
    ID_SIMULATION_CODE = 1001
    FCN_SIMULATION_CODE_RESET = 200
    
    ID_UNKNOWN = 0
    
    # Common
    FCN_UNKNOWN = 0
    FCN_REQUEST_PING = 1
    FCN_RESPONSE_PING = 2
    FCN_REQUEST_WORLD_TRANSFORM = 3
    FCN_RESPONSE_WORLD_TRANSFORM = 4
    

    
    # Initilize class
    def __init__(self):

       return

######################### Common #########################

###### Send Functions

    def common_RequestPing(self, device_id, device_num):
        self.device_id = device_id
        self.device_number = device_num
        self.device_function = self.FCN_REQUEST_PING
        self.payload = bytearray()                
        self.container_size = 10 + len(self.payload)
        return self   
        
    def common_RequestWorldTransform(self, device_id, device_num):
        self.device_id = device_id
        self.device_number = device_num
        self.device_function = self.FCN_REQUEST_WORLD_TRANSFORM
        self.payload = bytearray()                
        self.container_size = 10 + len(self.payload)
        return self           
    
 
###### Receive Functions 
    def common_ResponseWorldTransform(self):
        pos_x = 0.0
        pos_y = 0.0
        pos_z = 0.0
        rot_x = 0.0
        rot_y = 0.0
        rot_z = 0.0
        scale_x = 0.0
        scale_y = 0.0
        scale_z = 0.0
                
        if (len(self.payload) == 36):
            pos_x, pos_y, pos_z, rot_x, rot_y, rot_z, scale_x, scale_y, scale_z,  = struct.unpack(">fffffffff", self.payload)
        
        return pos_x, pos_y, pos_z, rot_x, rot_y, rot_z, scale_x, scale_y, scale_z
    

######################### QArm ######################### 

###### Send Functions

    def qarm_CommandAndRequestState(self, device_num, base, shoulder, elbow, wrist, gripper, base_color_r, base_color_g, base_color_b, arm_brightness):
        self.device_id = self.ID_QARM
        self.device_number = device_num
        self.device_function = self.FCN_QARM_COMMAND_AND_REQUEST_STATE
        self.payload = bytearray(struct.pack(">fffffffff", base, shoulder, elbow, wrist, gripper, base_color_r, base_color_g, base_color_b, arm_brightness))
        self.container_size = 10 + len(self.payload)
        return self   
    
    def qarm_CommandBase(self, device_num, base):
        self.device_id = self.ID_QARM
        self.device_number = device_num
        self.device_function = self.FCN_QARM_COMMAND_BASE
        self.payload = bytearray(struct.pack(">f", base))
        self.container_size = 10 + len(self.payload)
        return self    

    def qarm_CommandShoulder(self, device_num, shoulder):
        self.device_id = self.ID_QARM
        self.device_number = device_num
        self.device_function = self.FCN_QARM_COMMAND_SHOULDER
        self.payload = bytearray(struct.pack(">f", shoulder))
        self.container_size = 10 + len(self.payload)
        return self         
        
    def qarm_CommandElbow(self, device_num, elbow):
        self.device_id = self.ID_QARM
        self.device_number = device_num
        self.device_function = self.FCN_QARM_COMMAND_ELBOW
        self.payload = bytearray(struct.pack(">f", elbow))
        self.container_size = 10 + len(self.payload)
        return self      

    def qarm_CommandWrist(self, device_num, wrist):
        self.device_id = self.ID_QARM
        self.device_number = device_num
        self.device_function = self.FCN_QARM_COMMAND_WRIST
        self.payload = bytearray(struct.pack(">f", wrist))
        self.container_size = 10 + len(self.payload)
        return self     

    def qarm_CommandGripper(self, device_num, gripper):
        self.device_id = self.ID_QARM
        self.device_number = device_num
        self.device_function = self.FCN_QARM_COMMAND_GRIPPER
        self.payload = bytearray(struct.pack(">f", gripper))
        self.container_size = 10 + len(self.payload)
        return self     
        
    def qarm_CommandBaseColor(self, device_num, base_color_r, base_color_g, base_color_b):
        self.device_id = self.ID_QARM
        self.device_number = device_num
        self.device_function = self.FCN_QARM_COMMAND_BASE_COLOR
        self.payload = bytearray(struct.pack(">fff", base_color_r, base_color_g, base_color_b))
        self.container_size = 10 + len(self.payload)
        return self

    def qarm_CommandArmBrightness(self, device_num, arm_brightness):
        self.device_id = self.ID_QARM
        self.device_number = device_num
        self.device_function = self.FCN_QARM_COMMAND_ARM_BRIGHTNESS
        self.payload = bytearray(struct.pack(">f", arm_brightness))
        self.container_size = 10 + len(self.payload)
        return self      

    def qarm_RequestSegmentCollisions(self, device_num):
        self.device_id = self.ID_QARM
        self.device_number = device_num
        self.device_function = self.FCN_QARM_REQUEST_SEGMENT_COLLISIONS
        self.payload = bytearray()  
        self.container_size = 10 + len(self.payload)
        return self          

    def qarm_RequestGripperObjectProperties(self, device_num):
        self.device_id = self.ID_QARM
        self.device_number = device_num
        self.device_function = self.FCN_QARM_REQUEST_GRIPPER_OBJECT_PROPERTIES
        self.payload = bytearray()  
        self.container_size = 10 + len(self.payload)
        return self        

    def qarm_RequestEndEffectorCollisionSpheres(self, device_num):
        self.device_id = self.ID_QARM
        self.device_number = device_num
        self.device_function = self.FCN_QARM_REQUEST_END_EFFECTOR_COLLISION_SPHERE
        self.payload = bytearray()  
        self.container_size = 10 + len(self.payload)
        return self 
        
 
###### Receive Functions 
    def qarm_ResponseState(self):
        base = 0.0
        shoulder = 0.0
        elbow = 0.0
        wrist = 0.0
        gripper = 0.0
        static_environment_collision = 0
        finger_pad_detection_right_proximal = 0
        finger_pad_detection_right_distal = 0
        finger_pad_detection_left_proximal = 0
        finger_pad_detection_left_distal = 0
        
        if (len(self.payload) == 25):
            base, shoulder, elbow, wrist, gripper, static_environment_collision, finger_pad_detection_right_proximal, finger_pad_detection_right_distal, finger_pad_detection_left_proximal, finger_pad_detection_left_distal,  = struct.unpack(">fffffBBBBB", self.payload)
        
        return base, shoulder, elbow, wrist, gripper, static_environment_collision, finger_pad_detection_right_proximal, finger_pad_detection_right_distal, finger_pad_detection_left_proximal, finger_pad_detection_left_distal
    
    def qarm_ResponseBase(self):
        base = 0.0
                
        if (len(self.payload) == 4):
            base, = struct.unpack(">f", self.payload)
        
        return base
        
    def qarm_ResponseShoulder(self):
        shoulder = 0.0
                
        if (len(self.payload) == 4):
            shoulder, = struct.unpack(">f", self.payload)
        
        return shoulder        
        
    def qarm_ResponseElbow(self):
        elbow = 0.0
                
        if (len(self.payload) == 4):
            elbow, = struct.unpack(">f", self.payload)
        
        return elbow          

    def qarm_ResponseWrist(self):
        wrist = 0.0
                
        if (len(self.payload) == 4):
            wrist, = struct.unpack(">f", self.payload)
        
        return wrist             
        
    def qarm_ResponseSegmentCollisions(self):
        yaw = False
        shoulder = False
        elbow = False
        wrist = False
        finger_lp = False
        finger_ld = False
        finger_rp = False
        finger_rd = False
        
        if (len(self.payload) == 1):
            contact_mask, = struct.unpack(">B", self.payload)
            
            if contact_mask & 1:
                yaw = True
                
            if contact_mask & 2:
                shoulder = True
        
            if contact_mask & 4:
                elbow = True
        
            if contact_mask & 8:
                wrist = True
        
            if contact_mask & 16:
                finger_lp = True
        
            if contact_mask & 32:
                finger_ld = True
        
            if contact_mask & 64:
                finger_rp = True
        
            if contact_mask & 128:
                finger_rd = True
        
        return yaw, shoulder, elbow, wrist, finger_lp, finger_ld, finger_rp, finger_rd
              
        
    def qarm_ResponseGripper(self):
        gripper = 0.0
        static_environment_collision = 0
        finger_pad_detection_right_proximal = 0
        finger_pad_detection_right_distal = 0
        finger_pad_detection_left_proximal = 0
        finger_pad_detection_left_distal = 0
        
        if (len(self.payload) == 9):
            gripper, static_environment_collision, finger_pad_detection_right_proximal, finger_pad_detection_right_distal, finger_pad_detection_left_proximal, finger_pad_detection_left_distal,  = struct.unpack(">fBBBBB", self.payload)
        
        return gripper, static_environment_collision, finger_pad_detection_right_proximal, finger_pad_detection_right_distal, finger_pad_detection_left_proximal, finger_pad_detection_left_distal
         
    def qarm_ResponseGripperObjectProperties(self):
        object_id = 0
        mass = 0.0
        properties = ""
        properties_size = 0
        
        if (len(self.payload) >= 9):
            object_id, mass, properties_size, = struct.unpack(">BfI", self.payload[0:9])
            
            
            if (properties_size > 0):
                properties = self.payload[9:(9+properties_size)].decode("utf-8")
        
        return object_id, mass, properties      

    def qarm_ResponseEndEffectorCollisionSpheres(self):
        
        target_x = 0
        target_y = 0
        target_z = 0
        
        actual_x = 0
        actual_y = 0
        actual_z = 0
        
        if (len(self.payload) == 24):
            target_x, target_y, target_z, actual_x, actual_y, actual_z, = struct.unpack(">ffffff", self.payload)
        
        return target_x, target_y, target_z, actual_x, actual_y, actual_z
        
           
    
######################### QBot 2e ######################### 

###### Send Functions

    def qbot2e_CommandAndRequestState(self, device_num, forward, turn):
        d = 0.235
        right_wheel_speed = forward + d*turn/2
        left_wheel_speed = forward - d*turn/2
    
        self.device_id = self.ID_QBOT
        self.device_number = device_num
        self.device_function = self.FCN_QBOT_COMMAND_AND_REQUEST_STATE
        self.payload = bytearray(struct.pack(">ff", right_wheel_speed,left_wheel_speed))
        self.container_size = 10 + len(self.payload)
        return self   
        
    def qbot2e_CommandAndRequestStateTank(self, device_num, right_wheel_speed, left_wheel_speed):
        self.device_id = self.ID_QBOT
        self.device_number = device_num
        self.device_function = self.FCN_QBOT_COMMAND_AND_REQUEST_STATE
        self.payload = bytearray(struct.pack(">ff", right_wheel_speed, left_wheel_speed))
        self.container_size = 10 + len(self.payload)
        return self          
        
    def qbot2e_RequestRGB(self, device_num):
        self.device_id = self.ID_QBOT
        self.device_number = device_num
        self.device_function = self.FCN_QBOT_REQUEST_RGB    
        self.payload = bytearray()                
        self.container_size = 10 + len(self.payload) 
        return self        
        
    def qbot2e_RequestDepth(self, device_num):
        self.device_id = self.ID_QBOT
        self.device_number = device_num
        self.device_function = self.FCN_QBOT_REQUEST_DEPTH    
        self.payload = bytearray()                
        self.container_size = 10 + len(self.payload) 
        return self           
    
 
###### Receive Functions 
    def qbot2e_ResponseState(self):
        world_x = 0.0
        world_y = 0.0
        world_z = 0.0
        
        forward_x = 0.0
        forward_y = 0.0
        forward_z = 0.0
        
        up_x = 0.0
        up_y = 0.0
        up_z = 0.0
        
        bumper_front = 0
        bumper_left = 0
        bumper_right = 0
        
        gyro = 0.0
        heading = 0.0
        
        encoder_left = 0
        encoder_right = 0
        
        if (len(self.payload) == 55):
            world_x, world_y, world_z, forward_x, forward_y, forward_z, up_x, up_y, up_z, bumper_front, bumper_left, bumper_right, gyro, heading, encoder_left, encoder_right, = struct.unpack(">fffffffffBBBffii", self.payload)
        
        return world_x, world_y, world_z, forward_x, forward_y, forward_z, up_x, up_y, up_z, bumper_front, bumper_left, bumper_right, gyro, heading, encoder_left, encoder_right
        
    def qbot2e_ResponseRGB(self):   
        # just assume this is a valid payload for now.
        
        return bytearray(self.payload[4:len(self.payload)])
        
    def qbot2e_ResponseDepth(self):   
        # just assume this is a valid payload for now.
        
        return bytearray(self.payload[4:len(self.payload)])        
    
    
######################### EMG Interface ######################### 

###### Send Functions

    def EMG_RequestState(self, device_num):
        self.device_id = self.ID_EMG_INTERFACE
        self.device_number = device_num
        self.device_function = self.FCN_EMG_REQUEST_STATE
        self.payload = bytearray()  
        self.container_size = 10 + len(self.payload)
        return self    
        
 ###### Receive Functions
 
    def EMG_ResponseState(self):
        left_emg = 0.0
        right_emg = 0.0
               
        if (len(self.payload) == 8):
            left_emg, right_emg, = struct.unpack(">ff", self.payload)
        
        return left_emg, right_emg          
    
   
######################### SRV02 Bottle Table ######################### 

###### Send Functions

    def srv02BottleTable_CommandSpeed(self, device_num, speed):
        self.device_id = self.ID_SRV02BOTTLETABLE
        self.device_number = device_num
        self.device_function = self.FCN_SRV02BT_COMMAND_SPEED
        self.payload = bytearray(struct.pack(">f", speed))
        self.container_size = 10 + len(self.payload)
        return self
        
    def srv02BottleTable_RequestEncoder(self, device_num):
        self.device_id = self.ID_SRV02BOTTLETABLE
        self.device_number = device_num
        self.device_function = self.FCN_SRV02BT_REQUEST_ENCODER 
        self.payload = bytearray()
        self.container_size = 10 + len(self.payload)   
        return self

    def srv02BottleTable_RequestTOF(self, device_num):
        self.device_id = self.ID_SRV02BOTTLETABLE
        self.device_number = device_num
        self.device_function = self.FCN_SRV02BT_REQUEST_TOF 
        self.payload = bytearray()        
        self.container_size = 10 + len(self.payload)           
        return self

    def srv02BottleTable_RequestProximityShort(self, device_num):
        self.device_id = self.ID_SRV02BOTTLETABLE
        self.device_number = device_num
        self.device_function = self.FCN_SRV02BT_REQUEST_PROXIMITY_SHORT    
        self.payload = bytearray()                
        self.container_size = 10 + len(self.payload) 
        return self
        
    def srv02BottleTable_RequestProximityTall(self, device_num):
        self.device_id = self.ID_SRV02BOTTLETABLE
        self.device_number = device_num
        self.device_function = self.FCN_SRV02BT_REQUEST_PROXIMITY_TALL  
        self.payload = bytearray()                
        self.container_size = 10 + len(self.payload)         
        return self
        
    def srv02BottleTable_SpawnContainer(self, device_num, height, diameter, metallic, color_r, color_g, color_b, color_a, roughness, mass, properties):
        self.device_id = self.ID_SRV02BOTTLETABLE
        self.device_number = device_num
        self.device_function = self.FCN_SRV02BT_SPAWN_CONTAINER  
        
        encoded_string = bytearray(properties, 'utf-8')
        
        self.payload = bytearray(struct.pack(">ffBffffffI", height, diameter, metallic, color_r, color_g, color_b, color_a, roughness, mass, len(encoded_string))) + encoded_string
        self.container_size = 10 + len(self.payload)         
        return self       

    def srv02BottleTable_RequestLoadMass(self, device_num):
        self.device_id = self.ID_SRV02BOTTLETABLE
        self.device_number = device_num
        self.device_function = self.FCN_SRV02BT_REQUEST_LOAD_MASS
        self.payload = bytearray()
        self.container_size = 10 + len(self.payload)
        return self
       

        
###### Receive Functions
    def srv02BottleTable_ResponseEncoder(self):
        encoder_value = 0
    
        if (len(self.payload) == 4):
            encoder_value, = struct.unpack(">i", self.payload)
        
        return encoder_value
        
    def srv02BottleTable_ResponseTOF(self):
        tof_distance = 0.0
        
        if (len(self.payload) == 4):
            tof_distance, = struct.unpack(">f", self.payload)
        
        return tof_distance      

    def srv02BottleTable_ResponseProximityShort(self):
        relative_x = 0.0
        relative_y = 0.0
        relative_z = 0.0
        properties = ""
        properties_size = 0
        
        if (len(self.payload) >= 16):
            relative_x, relative_y, relative_z, properties_size, = struct.unpack(">fffI", self.payload[0:16])
            
            
            if (properties_size > 0):
                properties = self.payload[16:(16+properties_size)].decode("utf-8")
        
        return relative_x, relative_y, relative_z, properties
        
    def srv02BottleTable_ResponseProximityTall(self):
        relative_x = 0.0
        relative_y = 0.0
        relative_z = 0.0
        properties = ""
        properties_size = 0
        
        if (len(self.payload) >= 16):
            relative_x, relative_y, relative_z, properties_size, = struct.unpack(">fffI", self.payload[0:16])
            
            
            if (properties_size > 0):
                properties = self.payload[16:(16+properties_size)].decode("utf-8")
        
        return relative_x, relative_y, relative_z, properties    

    def srv02BottleTable_ResponesLoadMass(self):
        mass = 0
    
        if (len(self.payload) == 4):
            mass, = struct.unpack(">f", self.payload)
        
        return mass
  
 
 
 ######################### Scale ######################### 

###### Send Functions

    def scale_RequestLoadMass(self, device_num):
        self.device_id = self.ID_SCALE
        self.device_number = device_num
        self.device_function = self.FCN_SCALE_REQUEST_LOAD_MASS
        self.payload = bytearray()
        self.container_size = 10 + len(self.payload)
        return self
       
        
###### Receive Functions
    def scale_ResponesLoadMass(self):
        mass = 0
    
        if (len(self.payload) == 4):
            mass, = struct.unpack(">f", self.payload)
        
        return mass
   
 
 
######################### QBot 2e Box ######################### 

###### Send Functions

    def qbot2eBox_Command(self, device_num, x, y, z, roll, pitch, yaw):
        self.device_id = self.ID_QBOT_BOX
        self.device_number = device_num
        self.device_function = self.FCN_QBOT_BOX_COMMAND
        self.payload = bytearray(struct.pack(">ffffff", x, y, z, roll, pitch, yaw))
        self.container_size = 10 + len(self.payload)
        return self   
    
 
######################### Generic Spawner ######################### 

###### Send Functions

    def genericSpawner_Spawn(self, device_num, spawn_type):
        self.device_id = self.ID_GENERIC_SPAWNER
        self.device_number = device_num
        self.device_function = self.FCN_GENERIC_SPAWNER_SPAWN
        self.payload = bytearray(struct.pack(">B", spawn_type))
        self.container_size = 10 + len(self.payload)
        return self 
        
    def genericSpawner_Spawn_with_Properties(self, device_num, spawn_type, mass, properties):
        self.device_id = self.ID_GENERIC_SPAWNER
        self.device_number = device_num
        self.device_function = self.FCN_GENERIC_SPAWNER_SPAWN_WITH_PROPERTIES  
        
        encoded_string = bytearray(properties, 'utf-8')
        
        self.payload = bytearray(struct.pack(">BfI", spawn_type, mass, len(encoded_string))) + encoded_string
        self.container_size = 10 + len(self.payload)         
        return self         

###### Receive Functions
    def genericSpawner_SpawnAck(self):
        success = 0
        
        if (len(self.payload) == 1):
            success, = struct.unpack(">B", self.payload)
        
        return success    


######################### Autoclave ######################### 

###### Send Functions

    def autoclave_OpenDrawer(self, device_num, open_drawer):
        self.device_id = self.ID_AUTOCLAVE
        self.device_number = device_num
        self.device_function = self.FCN_AUTOCLAVE_OPEN_DRAWER
        self.payload = bytearray(struct.pack(">B", open_drawer))
        self.container_size = 10 + len(self.payload)
        return self 

###### Receive Functions



######################### Smart Box ######################### 

###### Send Functions



    def smartbox_Request_Surface_Properties(self, device_num):
        self.device_id = self.ID_SMARTBOX
        self.device_number = device_num
        self.device_function = self.FCN_SMARTBOX_REQUEST_SURFACE_PROPERTIES
        self.payload = bytearray()
        self.container_size = 10 + len(self.payload)
        return self 
             

###### Receive Functions
    def smartbox_Response_Surface_Properties(self):
            
        r = 0.0
        g = 0.0
        b = 0.0
        
        metallic = 0
        roughness = 0.0
       
        
        if (len(self.payload) == 17):
            r, g, b, metallic, roughness, = struct.unpack(">fffBf", self.payload)
        
        return r, g, b, metallic, roughness
       
       
 
       
######################### MODULAR COMMUNICATIONS CLASS ######################### 

class comm_modular_server:

    # Define class-level variables   
    _server_stream = None
    _client_connection = None

    _BUFFER_SIZE = 65537
    _read_buffer = bytearray(_BUFFER_SIZE)
    _send_buffer = bytearray()
#    _new_packet = True   


    _receive_packet_buffer = bytearray()
    _receive_packet_size = 0
    _receive_packet_container_index = 0

    # Initilize class
    def __init__(self,port):
       # Establish a server stream
        self._server_stream = Stream()

        self._server_stream.listen("{}{}".format("tcpip://localhost:",port), True)

        # Wait for incoming client connection
        print ("Waiting for simulation to connect...")
        
        poll_result = self._server_stream.poll(Timeout(3), PollFlag.ACCEPT)

        while poll_result & PollFlag.ACCEPT != PollFlag.ACCEPT:
            poll_result = self._server_stream.poll(Timeout(3), PollFlag.ACCEPT)


        if poll_result & PollFlag.ACCEPT == PollFlag.ACCEPT:
            print("Connection accepted")
        else:
            print("Connection timeout")
        
        
        self._client_connection = self._server_stream.accept(1460, self._BUFFER_SIZE)
        print ("Simulation connected")
        
    
    # Pack data and send immediately
    def send_container (self, container):
        try:
            data = bytearray(struct.pack("<i", 1+container.container_size)) + bytearray(struct.pack(">BiiBB", 123, container.container_size, container.device_id, container.device_number, container.device_function)) + container.payload
            num_bytes = len(data)
            bytes_written = self._client_connection.send(data, num_bytes)
            self._client_connection.flush()
        except:
            return
            
    # Build send buffer
    def queue_container (self, container):
        self._send_buffer = self._send_buffer + bytearray(struct.pack(">iiBB", container.container_size, container.device_id, container.device_number, container.device_function)) + container.payload
        #print("{} bytes added to queue.".format(len(self._send_buffer)))
        return            

    # Transmit send buffer
    def send_queue (self):
    
        if (len(self._send_buffer) > 0):
            #print("Sending queue of {} bytes.".format(len(self._send_buffer)))
    
            self._send_buffer = bytearray(struct.pack("<iB", 1+len(self._send_buffer), 123)) + self._send_buffer
            
            num_bytes = len(self._send_buffer)
            
            try:
            
                bytes_written = self._client_connection.send(self._send_buffer, num_bytes)
                self._client_connection.flush()  
                self._send_buffer = bytearray()   
                return bytes_written
            except StreamError as e:
                return e.error_code
        else:
            return 0


    # Check if new data is available.  Returns true if a complete packet has been received.
    def receive_new_data(self):    
        bytes_read = 0
        
        try:
            bytes_read = self._client_connection.receive(self._read_buffer, self._BUFFER_SIZE)
        except StreamError as e:
            if e.error_code == -34:
                # would block
                bytes_read = 0
        #print("Bytes read: {}".format(bytes_read))
            
        new_data = False

    
        while bytes_read > 0:
            #print("Received {} bytes".format(bytes_read))
            self._receive_packet_buffer += bytearray(self._read_buffer[0:(bytes_read)])

            #while we're here, check if there are any more bytes in the receive buffer
            try:
                bytes_read = self._client_connection.receive(self._read_buffer, self._BUFFER_SIZE)
            except StreamError as e:
                if e.error_code == -34:
                    # would block
                    bytes_read = 0
                    
        # check if we already have data in the receive buffer that was unprocessed (multiple packets in a single receive)
        if len(self._receive_packet_buffer) > 5:
            if (self._receive_packet_buffer[4] == 123):
                
                # packet size
                self._receive_packet_size, = struct.unpack("<I", self._receive_packet_buffer[0:4])
                # add the 4 bytes for the size to the packet size
                self._receive_packet_size = self._receive_packet_size + 4
            
            
                if len(self._receive_packet_buffer) >= self._receive_packet_size:
                    
                    self._receive_packet_container_index = 5
                    new_data = True
                   
            else:
                print("Error parsing multiple packets in receive buffer.  Clearing internal buffers.")
                _receive_packet_buffer = bytearray()
                
        return new_data



    # Parse out received containers
    def get_next_container(self):
        c = comm_modular_container()
        is_more_containers = False
    
        if (self._receive_packet_container_index > 0):
            c.container_size, = struct.unpack(">I", self._receive_packet_buffer[self._receive_packet_container_index:(self._receive_packet_container_index+4)])
            c.device_id, = struct.unpack(">I", self._receive_packet_buffer[(self._receive_packet_container_index+4):(self._receive_packet_container_index+8)])
            c.device_number = self._receive_packet_buffer[self._receive_packet_container_index+8]
            c.device_function = self._receive_packet_buffer[self._receive_packet_container_index+9]
            c.payload = bytearray(self._receive_packet_buffer[(self._receive_packet_container_index+10):(self._receive_packet_container_index+c.container_size)])
            
            self._receive_packet_container_index = self._receive_packet_container_index + c.container_size
            
            if (self._receive_packet_container_index >= self._receive_packet_size):
                
                is_more_containers = False
                
                if len(self._receive_packet_buffer) == self._receive_packet_size:
                    # The data buffer contains only the one packet.  Clear the buffer.
                    self._receive_packet_buffer = bytearray()
                else:
                    # Remove the packet from the data buffer.  There is another packet in the buffer already.
                    self._receive_packet_buffer = self._receive_packet_buffer[(self._receive_packet_container_index):(len(self._receive_packet_buffer))]
                    
                self._receive_packet_container_index = 0
                
            else:
                is_more_containers = True
                
    
        return c, is_more_containers
        
    

    ################## DEVICE MANAGEMENT ##################
    
    # Close Comm Server
    def close (self):
        self._server_stream.shutdown()
        self._server_stream.close()
        print ("Comm Server Closed")

