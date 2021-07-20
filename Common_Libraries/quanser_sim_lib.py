from quanser.communications import Stream, StreamError, PollFlag, Timeout
from quanser.common import GenericError
import struct
import math
import numpy as np
import sys
import time
import cv2
sys.path.append('../')
from Common_Libraries.modular_comm import comm_modular_container
from Common_Libraries.postman import postman
from Common_Libraries.quanser_image_lib import *


########################################################################
######################### VIRTUAL QBOT CLASSES ######################### 
########################################################################


############################## NEW QBOT SIM ############################
class QBot2e_sim:

    # Define class-level variables 
    _comms = None
    _comc = comm_modular_container()
    _inbox = []
    _qbot_diameter = 0.235

    _RGB_buffer = cv2.imread('DefaultImage.jpg')
    _depth_buffer = cv2.imread('DefaultImage.jpg')
    _gyro = 0
    _bumpers = [0,0,0]
    _world_xyz = [0,0,0]
    _fwd_xyz = [0,0,0]
    _up_xyz = [0,0,0] 

    _RGB_pending = False
    _CMD_pending = False
    _depth_pending = False
    _box_pending = False
    
    # Initilize QBot2e
    def __init__(self, postman, device_num = 0):
        self._comms = postman
        self._dev_num = device_num
        #self._flush()
        #self._set_box_angle(0)
        #self._request_RGB()
        #while not self._request_RGB():
        #    self._refresh()
        #self._command()
        #while not self._command():
        #    self._refresh()
        print("QBot2e Initialized")
    
    ##-------------------- COMM METHODS ----------------------

    def _refresh(self):
        # time.sleep (0.1)   # adding sleep may to improve line following
        self._comms.deliver()
        inbox = self._check_mail()
        if len(inbox) > 0:       
            for c in inbox:
                self._parse_container(c)

    def _flush(self):
        self._comms.flush()
    
    def _request_RGB(self):
        if not self._RGB_pending:
            self._comms.postMail(self._comc.qbot2e_RequestRGB(self._dev_num))
            self._refresh()
            self._RGB_pending = True
            return True
        else:
            return False

    def _request_depth(self):
        if not self._depth_pending:
            self._comms.postMail(self._comc.qbot2e_RequestDepth(self._dev_num))
            self._refresh()
            self._depth_pending = True
            return True
        else:
            return False

    def _command(self, speed = 0, turn = 0):
        if not self._CMD_pending:
            self._comms.postMail(self._comc.qbot2e_CommandAndRequestState(self._dev_num, speed, turn))
            self._comms.deliver()
            self._CMD_pending = True
            return True
        else:
            return False

    def _box_command(self, x = 0, y = 0, z = 0, x_r = 0, y_r = 0, z_r = 0):
        if not self._box_pending:
            self._comms.postMail(self._comc.qbot2eBox_Command(self._dev_num, x, y, z, x_r, y_r, z_r))
            self._refresh()
            self._box_pending = True
            return True
        else:
            return False
            
    def _check_mail(self):
        self._comms.fetch()
        inbox = self._comms.checkMail(comm_modular_container.ID_QBOT, self._dev_num)
        inbox += self._comms.checkMail(comm_modular_container.ID_QBOT_BOX, self._dev_num)
        return inbox
        
    def _parse_container(self, c_in):
        if c_in.device_function == comm_modular_container.FCN_QBOT_RESPONSE_STATE:
            #print("status container received")
            state = c_in.qbot2e_ResponseState()
            self._world_xyz = state[0:3]
            self._fwd_xyz = state[3:6]
            self._up_xyz = state[6:9]
            self._bumpers = state[9:12]
            self._gyro = state[12]
            self._CMD_pending = False
        elif c_in.device_function == comm_modular_container.FCN_QBOT_RESPONSE_RGB:
            #print("rgb container received")
            self._RGB_buffer = cv2.imdecode(np.frombuffer(c_in.qbot2e_ResponseRGB(), dtype=np.uint8, count=-1, offset=0), 1)
            self._RGB_pending = False
        elif c_in.device_function == comm_modular_container.FCN_QBOT_RESPONSE_DEPTH:
            self._depth_buffer = cv2.imdecode(np.frombuffer(c_in.qbot2e_ResponseDepth(), dtype=np.uint8, count=-1, offset=0), 1)
            self._depth_pending = False
        elif c_in.device_function == comm_modular_container.FCN_QBOT_BOX_COMMAND_ACK:
            self._box_pending = False

    ##------------------- SENSE METHODS ----------------------

    def get_bumpers(self):
        self._refresh()
        self._command()
        return self._bumpers

    def get_gyro(self):
        self._refresh()
        self._command()
        return self._gyro

    def get_position(self):
        self._refresh()
        self._command()
        return self._world_xyz
        
    def get_forward(self):
        self._refresh()
        self._command()
        return self._fwd_xyz        

    ##------------------- MSC METHODS ----------------
    def ping(self):
        # Creat ping request packet
        modc = comm_modular_container()
        self._comms.postMail(modc.common_RequestPing(0, 0))

        #Send all data back to QIL
        self._comms.deliver()

    ##------------------- CONTROL METHODS ----------------------

    # Set motor speed command
    def set_velocity(self, velocity = [0, 0]):
        speed = sum(velocity)/2
        turn = (velocity[1] - velocity[0])/self._qbot_diameter
        ref_count = 0
        while self._CMD_pending:
            ref_count += 1
            self._refresh()
            time.sleep(0.01)
            if ref_count > 10:
                #print("CMD ACK Missed")
                self._CMD_pending = False
                break
        self._command(speed, turn)

    def move_time(self, velocity = [0,0], t_finish = 0):
        t_start = time.perf_counter()
        t_delta = 0
        while t_delta < t_finish:
            self.set_velocity(velocity)
            t_now = time.perf_counter()
            t_delta = t_now - t_start
            time.sleep(0.05)
        self.halt()
    
    # Stop both motors
    def halt(self):
        #clear pending flag to force immediate command
        self._CMD_pending = False
        self.set_velocity([0, 0])

    ##---------------------- BOX METHODS -----------------------

    def _set_box_attitude(self, position = [0,0,0], rotation = [0,0,0]):
        x, y, z = position
        x_r, y_r, z_r = rotation
        ref_count = 0
        while self._box_pending:
            ref_count += 1
            self._refresh()
            time.sleep(0.01)
            if ref_count > 10:
                #print("BOX ACK Missed")
                self._box_pending = False
                break
        self._box_command(x, y, z, x_r, y_r, z_r)

    def _set_box_angle(self, theta):
        position = [0, (1-math.cos(theta)), math.sin(theta)]
        position = [x * 0.15 for x in position]
        rotation = [theta, 0, 0]
        self._set_box_attitude(position, rotation)

    def dump(self):
        for i in range(100):
            j = (float(i)/100.0)*math.tau
            theta = 1-math.cos(j)
            self._set_box_angle(theta)
 
    ##-------------------- CAMERA METHODS ----------------------

    # Get last RGB frame
    def get_RGB(self):
        self._refresh()
        self._request_RGB()
        return self._RGB_buffer

    # Get a new frame now
    def get_new_RGB(self):
        if not self._RGB_pending:
            self._request_RGB()
        while self._RGB_pending:
            self._refresh()
        return self._RGB_buffer

    # Get last depth frame
    def get_depth(self):
        self._refresh()
        self._request_depth()
        return self._depth_buffer

    # Get a new frame now
    def get_new_depth(self):
        if not self._depth_pending:
            self._request_depth()
        while self._depth_pending:
            self._refresh()
        self._request_depth()
        return self._depth_buffer
        
    # Return single point at location (row, column) depth measurement in meters
    def measure_depth (self, row = 240, col = 320):
        # Get last depth frame
        depth_frame = self.get_new_depth()
        
        # Extract central point; frame size 640 by 480; pixel values 0-255 (0~9.44 m)
        d = depth_frame[row][col][1]
        
        # Convert to m and return value
        d_meters = 9.44*d/255
        
        return d_meters

class CameraUI:

    #Define a camera UI using openCV for doing line following
    _max_speed = 0
    _hue_ctr = 49
    _hue_width = 17
    _ROI_x = [0,640]
    _ROI_y = [0,480]

    def __init__(self, look_ahead = 0.2, ROI_height = 32):

        y_min = round(480 * (1 - look_ahead))
        self._ROI_x = [0, 640]
        self._ROI_y = [y_min - ROI_height, y_min]
        # MARTHA COMMENTED OUT ALL ITEMS HERE. THAT POP UP SLIDERS ETC.
        #cv2.startWindowThread()
        #cv2.namedWindow('rgb_stream', cv2.WINDOW_AUTOSIZE)
        #cv2.namedWindow('binary_ROI', cv2.WINDOW_AUTOSIZE)
        img_RGB = cv2.imread('DefaultImage.jpg')
        #cv2.imshow('rgb_stream', img_RGB)
        img_binary = crop_rect(img_RGB, self._ROI_x, self._ROI_y)
        #cv2.imshow('binary_ROI', img_binary)

        #cv2.createTrackbar("Hue Center", 'binary_ROI', self._hue_ctr, 360, self._on_center)
        #cv2.createTrackbar("Hue Width", 'binary_ROI', self._hue_width, 180, self._on_width)
        #cv2.createTrackbar("Speed (mm/s)", 'binary_ROI', self._max_speed, 500, self._on_speed)
        
    def _on_center(self, val):
        self._hue_ctr = val

    def set_center(self, val):
        cv2.setTrackbarPos("Hue Center", 'binary_ROI', val)
        self._on_center(val)

    def _on_width(self, val):
        self._hue_width = val

    def set_width(self, val):
        cv2.setTrackbarPos("Hue Width", 'binary_ROI', val)
        self._on_width(val)

    def _on_speed(self, val):
        self._max_speed = val/1000

    def set_speed(self, val):
        cv2.setTrackbarPos("Speed (mm/s)", 'binary_ROI', val)
        self._on_speed(val)

    def process(self, img_RGB):
        #Threshold image for a given hue center and width
        img_buffer = img_RGB
        img_bin = hue_threshold(img_RGB, self._hue_ctr, self._hue_width, 360)

        #Crop thresholded image to ROI and show
        img_cropped = crop_rect(img_bin, self._ROI_x, self._ROI_y)
        #cv2.imshow('binary_ROI', img_cropped) # MARTHA COMMENTED OUT

        #Find center of line segment in ROI
        line_ctr = extract_line_ctr(img_cropped)

        #Overlay ROI and line location on image and show
        img_overlay = show_ROI_target(img_buffer, self._ROI_x, self._ROI_y, line_ctr)
        #cv2.imshow('rgb_stream', img_overlay) # MARTHA COMMENTED OUT. THIS IS THE ONE YOU WANT TO LOOK AT FOR VERIFICATION

        return line_ctr

    def get_ROI(self):
        return self._ROI_x, self._ROI_y

    def get_hue(self):
        return self._hue_ctr, self._hue_width

    def get_speed_lim(self):
        return self._max_speed

    def destroy(self):
        print("Closing UI")
        cv2.destroyAllWindows()




########################################################################
######################### VIRTUAL QARM CLASSES ######################### 
########################################################################

############################## NEW QARM SIM ############################
class QArm_sim:

    # Define class-level variables 
    base = 0
    shoulder = 0
    elbow = 0
    wrist = 0
    gripper = 0
    contact = 0
    contact_id = 0
    static_environment_collision = 0
    finger_pad_detection_right_proximal = 0
    finger_pad_detection_right_distal = 0
    finger_pad_detection_left_proximal = 0
    finger_pad_detection_left_distal = 0
    
    object_id = 0
    object_mass = 0.0
    object_properties = ""
    
    _comms = None
    _comc = comm_modular_container()
    _dev_num = 0

    # Manipulator parameters in meters:
    _L1 = 0.127
    _L2 = 0.3556
    _L3 = 0.4064

    # Define joint angle (in rads) and gripper limits
    _qarm_base_upper_lim = 3.05
    _qarm_base_lower_lim = -3.05
    _qarm_shoulder_upper_limit = 1.57
    _qarm_shoulder_lower_limit = -1.57
    _qarm_elbow_upper_limit = 1.57
    _qarm_elbow_lower_limit = -1.39
    _qarm_wrist_upper_limit = 2.96
    _qarm_wrist_lower_limit = -2.96
    _qarm_gripper_upper_limit = 1
    _qarm_gripper_lower_limit = 0
    
    # Define base LED color
    _base_color_r = 1
    _base_color_g = 0
    _base_color_b = 0
        
    _arm_brightness = 1 
        

    image_rgb = None
    image_depth = None
           
    # Initilize QuanserSim
    def __init__(self, postman, device_num = 0):
        self._comms = postman
        self._dev_num = device_num
        print ("Virtual QArm initialized")

    # Set base LED color; color is an array of 3 elements of [r, g, b]; element values from 0-1
    def set_base_color (self, color):
        self._base_color_r = color[0]
        self._base_color_g = color[1]
        self._base_color_b = color[2]
        
        modc = comm_modular_container()
        self._comms.postMail(modc.qarm_CommandBaseColor(self._dev_num, self._base_color_r, self._base_color_g, self._base_color_b))
        self._comms.deliver()        
        
    def return_home(self):
            modc = comm_modular_container()
            self._comms.postMail(modc.qarm_CommandAndRequestState(0, 0, 0, 0, 0, 0, self._base_color_r, self._base_color_g, self._base_color_b, 0))
            self._comms.deliver()
            time.sleep(0.1)
    
    
    def check_any_collision(self):
        yaw, shoulder, elbow, wrist, finger_lp, finger_ld, finger_rp, finger_rd = self.check_collision()
        
        if (yaw or shoulder or elbow or wrist or finger_lp or finger_ld or finger_rp or finger_rd):
            return True
        else:
            return False
    
    
    def check_collision(self):
    
        yaw = False
        shoulder = False
        elbow = False
        wrist = False
        finger_lp = False
        finger_ld = False
        finger_rp = False
        finger_rd = False
    
        modc = comm_modular_container()

        #Request sensor feedback; read values on next fetch
        self._comms.postMail(modc.qarm_RequestSegmentCollisions(self._dev_num))
        self._comms.deliver()
        time.sleep(0.01)
        
        #Extract emg containers
        TT_In = self._comms.checkMail(modc.ID_QARM)
                
        while(len(TT_In) == 0): 
            #Fetch until new data is received from simulation
            count = self._comms.fetch()
            time.sleep(0.01)

            #Extract emg containers
            TT_In = self._comms.checkMail(modc.ID_QARM)  
            
        ###Parse each container###
        if len(TT_In) > 0:
            for TT_container in TT_In:
                if TT_container.device_function == comm_modular_container.FCN_QARM_RESPONSE_SEGMENT_COLLISIONS: 
                    yaw, shoulder, elbow, wrist, finger_lp, finger_ld, finger_rp, finger_rd = TT_container.qarm_ResponseSegmentCollisions()
                    
        return yaw, shoulder, elbow, wrist, finger_lp, finger_ld, finger_rp, finger_rd
        
        
    def get_collision_spheres(self):
    
        target_x = 0
        target_y = 0
        target_z = 0
        
        actual_x = 0
        actual_y = 0
        actual_z = 0
    
        modc = comm_modular_container()

        #Request sensor feedback; read values on next fetch
        self._comms.postMail(modc.qarm_RequestEndEffectorCollisionSpheres(self._dev_num))
        self._comms.deliver()
        time.sleep(0.01)
        
        #Extract emg containers
        TT_In = self._comms.checkMail(modc.ID_QARM)
                
        while(len(TT_In) == 0): 
            #Fetch until new data is received from simulation
            count = self._comms.fetch()
            time.sleep(0.01)

            #Extract emg containers
            TT_In = self._comms.checkMail(modc.ID_QARM)  
            
        ###Parse each container###
        if len(TT_In) > 0:
            for TT_container in TT_In:
                if TT_container.device_function == comm_modular_container.FCN_QARM_RESPONSE_END_EFFECTOR_COLLISION_SPHERE: 
                    target_x, target_y, target_z, actual_x, actual_y, actual_z = TT_container.qarm_ResponseEndEffectorCollisionSpheres()
                    
        target_array = [target_x, target_y, target_z]
        actual_array = [actual_x, actual_y, actual_z]
                    
        return target_array, actual_array
    
    
    
    # All angles in rads
    def qarm_move(self, base, shoulder, elbow, wrist, gripper, wait = True, tolerance = 0.002, stop_on_collision = False):
        modc = comm_modular_container()

        self._comms.postMail(modc.qarm_CommandAndRequestState(self._dev_num, base, shoulder, elbow, wrist, gripper, self._base_color_r, self._base_color_g, self._base_color_b, self._arm_brightness))
        self._comms.deliver()
        
        if (wait == True):
            b, s, e, w, g = self.read_all_arm_joints()
            
            if (stop_on_collision):
                if self.check_any_collision():
                    self._comms.postMail(modc.qarm_CommandAndRequestState(self._dev_num, b, s, e, w, g, self._base_color_r, self._base_color_g, self._base_color_b, self._arm_brightness))
                    self._comms.deliver()    
                    
                    b, s, e, w, g = self.read_all_arm_joints()
                    
                    return b, s, e, w, g, True
        
            reached = False
            while not reached:
                if ((abs(b - base) < tolerance) and (abs(s - shoulder) < tolerance) and (abs(e - elbow) < tolerance) and (abs(w - wrist) < tolerance) and (abs(g - gripper) < tolerance)):
                    reached = True
                else:
                    self._comms.postMail(modc.qarm_CommandAndRequestState(self._dev_num, base, shoulder, elbow, wrist, gripper, self._base_color_r, self._base_color_g, self._base_color_b, self._arm_brightness))
                    self._comms.deliver()
                    
                    b, s, e, w, g = self.read_all_arm_joints()

                    
                    if (stop_on_collision):
                        if self.check_any_collision():
                            self._comms.postMail(modc.qarm_CommandAndRequestState(self._dev_num, b, s, e, w, g, self._base_color_r, self._base_color_g, self._base_color_b, self._arm_brightness))
                            self._comms.deliver()    
                            
                            b, s, e, w, g = self.read_all_arm_joints()
                            
                            return b, s, e, w, g, True 
            return b, s, e, w, g, False
        return 0
        
 
 
 
 
    def _qarm_collision_inverse_kinematics(self, p, phi_prev):

        gamma = 0
        
        # QUANSER_ARM_IPK
        # v 4.0 - 4th June 2020
        # REFERENCE: 
        # Chapter 4. Inverse Kinematics
        # Robot Dynamics and Control 
        # Spong, Vidyasagar
        # 1989
        # INPUTS:
        # p        : end-effector position vector expressed in base frame {0}
        # gamma    : wrist rotation angle gamma
        # OUTPUTS:
        # phi_optimal : Best solution depending on phi_prev
        # phi         : All four Inverse Kinematics solutions as a 4x4 matrix. 
        #               Each col is a solution.
        
        # Manipulator parameters:
        L1 = 0.1400
        L2 = 0.3500
        L3 = 0.0500
        L4 = 0.2500
        L5 = 0.1500
        beta = math.atan(L3/L2)
        
        # Alternate parameters
        lambda1 = L1
        lambda2 = math.sqrt(L2*L2 + L3*L3)
        lambda3 = L4 + L5
        
        # Initialization
        theta = np.array([[0., 0., 0., 0.], [0., 0., 0., 0.], [0., 0., 0., 0.], [0., 0., 0., 0.]])
        phi = np.array([[0., 0., 0., 0.], [0., 0., 0., 0.], [0., 0., 0., 0.], [0., 0., 0., 0.]])
        
        # Joint angle 2 and 3: 
        # Equations:
            # lambda2 cos(theta2) + (-lambda3) sin(theta2 + theta3) = sqrt(x^2 + y^2)
            #   A     cos( 2    ) +     C      sin(   2   +    3  ) =    D
            
            # lambda2 sin(theta2) - (-lambda3) cos(theta2 + theta3) = lambda1 - z
            #   A     sin( 2    ) -     C      cos(   2   +    3  ) =    H
        # Solution:
        A = lambda2
        C = -lambda3
        H =  lambda1 - p[2,0]
        D1 = -math.sqrt(p[0,0]*p[0,0] + p[1,0]*p[1,0])
        D2 = -D1
        F = (D1*D1 + H*H - A*A - C*C)/(2*A)
        
        # print("p: {}".format(p))
        
        # print("A: {}".format(A))
        # print("C: {}".format(C))
        # print("H: {}".format(H))
        # print("D1: {}".format(D1))
        # print("D2: {}".format(D2))
        # print("F: {}".format(F))
                
        theta[2,0] = 2*math.atan2( C + math.sqrt(C*C - F*F), F )
        theta[2,1] = 2*math.atan2( C - math.sqrt(C*C - F*F), F )
        theta[2,2] = 2*math.atan2( C + math.sqrt(C*C - F*F), F )
        theta[2,3] = 2*math.atan2( C - math.sqrt(C*C - F*F), F )


        # print("Theta: {}".format(theta))
        
        M = A + C*math.sin(theta[2,0])
        N = -C*math.cos(theta[2,0])
        cos_term = (D1*M + H*N)/(M*M + N*N)
        sin_term = (H - N*cos_term)/(M)
        theta[1,0] = math.atan2(sin_term, cos_term)
        
        M = A + C*math.sin(theta[2,1])
        N = -C*math.cos(theta[2,1])
        cos_term = (D1*M + H*N)/(M*M + N*N)
        sin_term = (H - N*cos_term)/(M)
        theta[1,1] = math.atan2(sin_term, cos_term)
        
        M = A + C*math.sin(theta[2,2])
        N = -C*math.cos(theta[2,2])
        cos_term = (D2*M + H*N)/(M*M + N*N)
        sin_term = (H - N*cos_term)/(M)
        theta[1,2] = math.atan2(sin_term, cos_term)
        
        M = A + C*math.sin(theta[2,3])
        N = -C*math.cos(theta[2,3])
        cos_term = (D2*M + H*N)/(M*M + N*N)
        sin_term = (H - N*cos_term)/(M)
        theta[1,3] = math.atan2(sin_term, cos_term)
        
        # Joint angle 1:
        # Equation:
            # y / x = tan(theta1);
        # Solution
        theta[0,0] = math.atan2( p[1]/( lambda2 * math.cos( theta[1,0] ) - lambda3 * math.sin( theta[1,0] + theta[2,0] ) ) , p[0]/(  lambda2 * math.cos( theta[1,0] ) - lambda3 * math.sin( theta[1,0] + theta[2,0] )  ) )
        theta[0,1] = math.atan2( p[1]/( lambda2 * math.cos( theta[1,1] ) - lambda3 * math.sin( theta[1,1] + theta[2,1] ) ) , p[0]/(  lambda2 * math.cos( theta[1,1] ) - lambda3 * math.sin( theta[1,1] + theta[2,1] )  ) )
        theta[0,2] = math.atan2( p[1]/( lambda2 * math.cos( theta[1,2] ) - lambda3 * math.sin( theta[1,2] + theta[2,2] ) ) , p[0]/(  lambda2 * math.cos( theta[1,2] ) - lambda3 * math.sin( theta[1,2] + theta[2,2] )  ) )
        theta[0,3] = math.atan2( p[1]/( lambda2 * math.cos( theta[1,3] ) - lambda3 * math.sin( theta[1,3] + theta[2,3] ) ) , p[0]/(  lambda2 * math.cos( theta[1,3] ) - lambda3 * math.sin( theta[1,3] + theta[2,3] )  ) )

        

        # Remap theta back to phi
        # phi(1) = theta(1);
        # phi(2) = theta(2) - beta + pi/2;
        # phi(3) = theta(3) + beta;
        # phi(4) = theta(4);
        phi[0] = theta[0]
        phi[1] = theta[1] - beta + math.pi/2
        phi[2] = theta[2] + beta
        phi[3] = np.array([gamma, gamma, gamma, gamma])
        # print("Phi: {}".format(phi))
        
        phi = np.mod(phi + math.pi, 2*math.pi) - math.pi
        

        
        phi_optimal = np.array([[phi[0,0]], [phi[1,0]], [phi[2,0]], [phi[3,0]]])
        
        if (np.linalg.norm(np.subtract(phi_optimal, phi_prev)) > np.linalg.norm(np.subtract(phi[:,1], phi_prev))):
            phi_optimal = np.array([[phi[0,1]], [phi[1,1]], [phi[2,1]], [phi[3,1]]])

        if (np.linalg.norm(np.subtract(phi_optimal, phi_prev)) > np.linalg.norm(np.subtract(phi[:,2], phi_prev))):
            phi_optimal = np.array([[phi[0,2]], [phi[1,2]], [phi[2,2]], [phi[3,2]]])

        if (np.linalg.norm(np.subtract(phi_optimal, phi_prev)) > np.linalg.norm(np.subtract(phi[:,3], phi_prev))):
            phi_optimal = np.array([[phi[0,3]], [phi[1,3]], [phi[2,3]], [phi[3,3]]])

        return phi_optimal
        
        
    def _qarm_collision_DH(self, a, alpha, d, theta):
        # QUANSER_ARM_DH
        # v 1.0 - 26th March 2019
        # REFERENCE: 
        # Chapter 3. Forward and Inverse Kinematics
        # Robot Modeling and Control 
        # Spong, Hutchinson, Vidyasagar
        # 2006
        # INPUTS:
        # a       :   translation  : along : x_{i}   : from : z_{i-1} : to : z_{i}
        # alpha   :      rotation  : about : x_{i}   : from : z_{i-1} : to : z_{i}
        # d       :   translation  : along : z_{i-1} : from : x_{i-1} : to : x_{i}
        # theta   :      rotation  : about : z_{i-1} : from : x_{i-1} : to : x_{i}
        # NOTE: Standard DH Parameters
        # OUTPUTS:
        # T       : transformation                   : from :     {i} : to : {i-1}
        
        # Rotation Transformation about z axis by theta
        #T_R_z = [cos(theta) -sin(theta) 0 0;
        #         sin(theta)  cos(theta) 0 0;
        #                  0           0 1 0;
        #                  0           0 0 1];
        
        T_R_z = np.array([[math.cos(theta), -math.sin(theta), 0, 0], [math.sin(theta), math.cos(theta), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        

        # Translation Transformation along z axis by d
        #T_T_z = [1 0 0 0;
        #         0 1 0 0;
        #         0 0 1 d;
        #         0 0 0 1];
        T_T_z = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, d], [0, 0, 0, 1]])
        

        # Translation Transformation along x axis by a
        #T_T_x = [1 0 0 a;
        #         0 1 0 0;
        #         0 0 1 0;
        #         0 0 0 1];
        T_T_x = np.array([[1, 0, 0, a], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

        # Rotation Transformation about x axis by alpha
        #T_R_x = [1          0           0 0;
        #         0 cos(alpha) -sin(alpha) 0;
        #         0 sin(alpha)  cos(alpha) 0;
        #         0          0           0 1];
        T_R_x = np.array([[1, 0, 0, 0], [0, math.cos(alpha), -math.sin(alpha), 0], [0, math.sin(alpha), math.cos(alpha), 0], [0, 0, 0, 1]])

        # For a transformation FROM frame {i} TO frame {i-1}: A
        T = np.matmul(T_R_z, T_T_z)
        T = np.matmul(T, T_T_x)
        T = np.matmul(T, T_R_x)
        
        return T

    def _qarm_collision_forward_kinematics(self, phi):
    
        # QUANSER_ARM_FPK
        # v 4.0 - 4th June 2020
        # REFERENCE: 
        # Chapter 3. Forward Kinematics
        # Robot Dynamics and Control 
        # Spong, Vidyasagar
        # 1989
        # INPUTS:
        # phi     : Alternate joint angles vector 4 x 1
        # OUTPUTS:
        # p4      : End-effector frame {4} position vector expressed in base frame {0}
        # R04     : rotation matrix from end-effector frame {4} to base frame {0}
        
        # Manipulator parameters:
        L1 = 0.1400
        L2 = 0.3500
        L3 = 0.0500
        L4 = 0.2500
        L5 = 0.1500
        
        # Alternate parameters
        l1 = L1
        l2 = math.sqrt(L2*L2 + L3*L3)
        l3 = L4 + L5
        beta = math.atan(L3/L2)
        
        # From phi space to theta space 
        theta1 = phi[0]
        theta2 = phi[1] + beta - math.pi/2
        theta3 = phi[2] - beta
        theta4 = phi[3]
        
        # Transformation matrices for all frames:
        # T{i-1}{i} = quanser_arm_DH(  a, alpha,  d,     theta );
        T01 = self._qarm_collision_DH(  0, -math.pi/2, l1,  theta1 )
        T12 = self._qarm_collision_DH( l2,     0,  0,  theta2 )
        T23 = self._qarm_collision_DH(  0, -math.pi/2,  0,  theta3 )
        T34 = self._qarm_collision_DH(  0,     0, l3,  theta4 )
        
        T02 = np.matmul(T01, T12)
        T03 = np.matmul(T02, T23)
        T04 = np.matmul(T03, T34)
        
        # Position of end-effector Transformation
        # Extract the Position vector 
        # p1   = T01(1:3,4);
        # p2   = T02(1:3,4);
        # p3   = T03(1:3,4);
        #p4   = T04(1:3,4);
        p4 = np.array([[T04[0,3]], [T04[1,3]], [T04[2,3]]])
        
        # Extract the Rotation matrix
        # R01 = T01(1:3,1:3);
        # R02 = T02(1:3,1:3);
        # R03 = T03(1:3,1:3);
        #R04 = T04(1:3,1:3)
        R04 = np.array([[T04[0,0], T04[0,1], T04[0,2]], [T04[1,0], T04[1,1], T04[1,2]], [T04[2,0], T04[2,1], T04[2,2]]])
         
        return p4, R04

    def _qarm_collision_optimal_position(self, user_phi, user_p, target, actual, R04, state, phi, actual_hold):
        
        end_effector_to_collision_transform = np.array([[-0.0256], [0], [-0.125]])
        
        
        change = 0
        collision = 0
        
        if (np.linalg.norm(np.subtract(user_phi,phi), 2) > 0.001):
            change = 1
        
        if (np.linalg.norm(np.subtract(target,actual), 2) > 0.001):
            collision = 1


        if (state == 1):
            if collision:
                state = 2
            else:
                state = 1

            position_cmd = user_p
            
        elif (state == 2):

            state = 3
            actual_hold = actual
            position_cmd = np.subtract(actual_hold, np.matmul(R04,end_effector_to_collision_transform))
            
        elif (state == 3):
            if change:
                state = 1
            else:
                state = 3

            position_cmd = np.subtract(actual_hold, np.matmul(R04,end_effector_to_collision_transform))
        
        else:
            position_cmd = user_p

        phi = user_phi
     
     
        return position_cmd, state, phi, actual_hold
      
    # All angles in rads
    def qarm_move_collision_avoidance(self, base, shoulder, elbow, wrist, gripper, change_threshold = 0.002):
        modc = comm_modular_container()


        delta = change_threshold + 1      
        
        modc = comm_modular_container()

        self._comms.postMail(modc.qarm_CommandBase(self._dev_num, base))
        self._comms.deliver()
        
        b_last, s_last, e_last, w_last, g_last = self.read_all_arm_joints()
        #ensure we get at least one full pass throught the kinematics
        g_last = g_last + 1000
        
        # initialize states
        user_phi = np.array([base, shoulder, elbow, wrist])
        
        user_endpoint, last_R04 = self._qarm_collision_forward_kinematics(user_phi)
        last_phi_optimal = np.array([[b_last], [s_last], [e_last], [w_last]])
        
        state = 1
        phi = user_phi
        actual_hold = np.array([[0], [0], [0]])
        
        
        # constants
        world_to_base_transform = np.array([[0, -1, 0, 0], [-1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        
        
        count = 0
        #while (delta > change_threshold):
        while (count < 5):
        
            # get current collision target and actual position spheres (visible with ghost option in dev builds)
            target_temp, actual_temp = self.get_collision_spheres()
           
            # convert to 4x1 np array and units to m from cm
            target_world = np.array([[target_temp[0]], [target_temp[1]], [target_temp[2]], [100]])*0.01
            actual_world = np.array([[actual_temp[0]], [actual_temp[1]], [actual_temp[2]], [100]])*0.01     
            
            # convert to base frame
            target_base4 = np.matmul(world_to_base_transform, target_world)
            actual_base4 = np.matmul(world_to_base_transform, actual_world)          
            
            # reshape to 1x3
            target_base3 = np.array([target_base4[0], target_base4[1], target_base4[2]])
            actual_base3 = np.array([actual_base4[0], actual_base4[1], actual_base4[2]])
            
            # calculate end-point
            position_cmd, state, phi, actual_hold = self._qarm_collision_optimal_position(user_phi, user_endpoint, target_base3, actual_base3, last_R04, state, phi, actual_hold)
            
            # calculate joint angles
            phi_optimal = self._qarm_collision_inverse_kinematics(position_cmd, last_phi_optimal)
            
            
            # calculate the forward kinematics for the next pass
            temp, last_R04 = self._qarm_collision_forward_kinematics(phi_optimal)
            
            # send joint angles
            self._comms.postMail(modc.qarm_CommandAndRequestState(self._dev_num, phi_optimal[0], phi_optimal[1], phi_optimal[2], phi_optimal[3], gripper, self._base_color_r, self._base_color_g, self._base_color_b, self._arm_brightness))
            #self._comms.postMail(modc.qarm_CommandAndRequestState(self._dev_num, last_phi_optimal[0], last_phi_optimal[1], last_phi_optimal[2], last_phi_optimal[3], gripper, self._base_color_r, self._base_color_g, self._base_color_b, self._arm_brightness))
            self._comms.deliver()    
            
            b, s, e, w, g = self.read_all_arm_joints()
            
            delta = abs(b - b_last) + abs(s - s_last) + abs(e - e_last) + abs(w - w_last) + abs(g - g_last)
            b_last = b
            s_last = s
            e_last = e
            w_last = w
            g_last = g
            
            last_phi_optimal = phi_optimal
            
            count = count + 1

            if (delta < change_threshold):
                break

        return b, s, e, w, g, False
       
       
        
    # All angles in rads
    def qarm_move_base(self, base, wait = True, tolerance = 0.002):
        modc = comm_modular_container()

        self._comms.postMail(modc.qarm_CommandBase(self._dev_num, base))
        self._comms.deliver()
        
        if (wait == True):
            
            b, s, e, w, g = self.read_all_arm_joints()
            
            reached = False
            while not reached:
                if ((abs(b - base) < tolerance)):
                    reached = True
                else:
                    self._comms.postMail(modc.qarm_CommandBase(self._dev_num, base))
                    self._comms.deliver()
                    time.sleep(0.1)
                    b, s, e, w, g = self.read_all_arm_joints()
            return b
        return 0
        
    def qarm_move_shoulder(self, shoulder, wait = True, tolerance = 0.002):
        modc = comm_modular_container()

        self._comms.postMail(modc.qarm_CommandShoulder(self._dev_num, shoulder))
        self._comms.deliver()
        
        if (wait == True):
            time.sleep(0.1)
            b, s, e, w, g = self.read_all_arm_joints()
            
            reached = False
            while not reached:
                if ((abs(s - shoulder) < tolerance)):
                    reached = True
                else:
                    self._comms.postMail(modc.qarm_CommandShoulder(self._dev_num, shoulder))
                    self._comms.deliver()
                    time.sleep(0.1)
                    b, s, e, w, g = self.read_all_arm_joints()
            return s   
        return 0
        
    def qarm_move_elbow(self, elbow, wait = True, tolerance = 0.002):
        modc = comm_modular_container()

        self._comms.postMail(modc.qarm_CommandElbow(self._dev_num, elbow))
        self._comms.deliver()
        
        if (wait == True):
            time.sleep(0.1)
            b, s, e, w, g = self.read_all_arm_joints()
            
            reached = False
            while not reached:
                if ((abs(e - elbow) < tolerance)):
                    reached = True
                else:
                    self._comms.postMail(modc.qarm_CommandElbow(self._dev_num, elbow))
                    self._comms.deliver()
                    time.sleep(0.1)
                    b, s, e, w, g = self.read_all_arm_joints()
            return e  
        return 0
        
    def qarm_move_wrist(self, wrist, wait = True, tolerance = 0.002):
        modc = comm_modular_container()

        self._comms.postMail(modc.qarm_CommandWrist(self._dev_num, wrist))
        self._comms.deliver()
        
        if (wait == True):
            time.sleep(0.1)
            b, s, e, w, g = self.read_all_arm_joints()
            
            reached = False
            while not reached:
                if ((abs(w - wrist) < tolerance)):
                    reached = True
                else:
                    self._comms.postMail(modc.qarm_CommandWrist(self._dev_num, wrist))
                    self._comms.deliver()
                    time.sleep(0.1)
                    b, s, e, w, g = self.read_all_arm_joints()
            return w  
        return 0
        
        
    def qarm_move_gripper(self, gripper, wait = True, tolerance = 0.002):
        modc = comm_modular_container()

        self._comms.postMail(modc.qarm_CommandGripper(self._dev_num, gripper))
        self._comms.deliver()
        
        if (wait == True):
            time.sleep(0.1)
            b, s, e, w, g = self.read_all_arm_joints()
            
            reached = False
            while not reached:
                if ((abs(g - gripper) < tolerance)):
                    reached = True
                else:
                    self._comms.postMail(modc.qarm_CommandGripper(self._dev_num, gripper))
                    self._comms.deliver()
                    time.sleep(0.1)
                    b, s, e, w, g = self.read_all_arm_joints()
            return g        
        return 0


                
                    
                                        
                    

    def read_all_arm_joints(self):
    
        modc = comm_modular_container()
        #Fetch until new QArm data is received from simulation
        processing = True
        while processing:
            time.sleep(0.01)
            count = self._comms.fetch()
            
            if (count > 0):
                QA_In = self._comms.checkMail(comm_modular_container.ID_QARM)
         


                for QA_container in QA_In:
                    if (QA_container.device_function == comm_modular_container.FCN_QARM_RESPONSE_STATE):
                        self.base, self.shoulder, self.elbow, self.wrist, self.gripper, self.static_environment_collision, \
                        self.finger_pad_detection_right_proximal, self.finger_pad_detection_right_distal, \
                        self.finger_pad_detection_left_proximal, self.finger_pad_detection_left_distal \
                        = QA_container.qarm_ResponseState()
                        processing = False
                     
                    elif (QA_container.device_function == comm_modular_container.FCN_QARM_RESPONSE_BASE):
                        self.base =  QA_container.qarm_ResponseBase()
                        processing = False

                    elif (QA_container.device_function == comm_modular_container.FCN_QARM_RESPONSE_SHOULDER):
                        self.shoulder =  QA_container.qarm_ResponseShoulder() 
                        processing = False                        

                    elif (QA_container.device_function == comm_modular_container.FCN_QARM_RESPONSE_ELBOW):
                        self.elbow =  QA_container.qarm_ResponseElbow()  
                        processing = False

                    elif (QA_container.device_function == comm_modular_container.FCN_QARM_RESPONSE_WRIST):
                        self.wrist =  QA_container.qarm_ResponseWrist()    
                        processing = False

                    elif (QA_container.device_function == comm_modular_container.FCN_QARM_RESPONSE_GRIPPER):
                        
                        self.gripper, self.static_environment_collision, \
                        self.finger_pad_detection_right_proximal, self.finger_pad_detection_right_distal, \
                        self.finger_pad_detection_left_proximal, self.finger_pad_detection_left_distal \
                        = QA_container.qarm_ResponseGripper()
                        processing = False
                
    
        return self.base, self.shoulder, self.elbow, self.wrist, self.gripper
        
    # Check if given joint angles and gripper value are within acceptable limit
    # Return 1 if withing bound, 0 otherwise
    def angles_within_bound (self, qarm_base, qarm_shoulder, qarm_elbow, qarm_wrist, qarm_gripper):
        if qarm_base > self._qarm_base_upper_lim or qarm_base < self._qarm_base_lower_lim or \
                qarm_shoulder > self._qarm_shoulder_upper_limit or qarm_shoulder < self._qarm_shoulder_lower_limit or \
                qarm_elbow > self._qarm_elbow_upper_limit or qarm_elbow < self._qarm_elbow_lower_limit or \
                qarm_wrist > self._qarm_wrist_upper_limit or qarm_wrist < self._qarm_wrist_lower_limit or \
                qarm_gripper > self._qarm_gripper_upper_limit or qarm_gripper < self._qarm_gripper_lower_limit:
            return 0
        else:
            return 1

    # Check if given end-effector coordinates are within bounds
    # Return 1 if withing bound, 0 otherwise
    def coordinates_within_bound(self, p_x, p_y, p_z):
        R = math.sqrt(p_x ** 2 + p_y ** 2)

        # Vertical offset within the verical plane from Frame 1 to End-Effector
        # Note: Frame 1 y-axis points downward (negative global Z-axis direction)
        Z = self._L1 - p_z

        # Distance from Frame 1 to End-Effector Frame
        Lambda = math.sqrt(R ** 2 + Z ** 2)

        if Lambda > (self._L2 + self._L3) or p_z < 0:
            return 0
        else:
            return 1

    # Calculate standard DH parameters
    # Inputs:
    # a       :   translation  : along : x_{i}   : from : z_{i-1} : to : z_{i}
    # alpha   :      rotation  : about : x_{i}   : from : z_{i-1} : to : z_{i}
    # d       :   translation  : along : z_{i-1} : from : x_{i-1} : to : x_{i}
    # theta   :      rotation  : about : z_{i-1} : from : x_{i-1} : to : x_{i}
    # Outputs:
    # transformed       : transformation                   : from :     {i} : to : {i-1}
    def qarm_dh(self, theta, d, a, alpha):
        # Rotation Transformation about z axis by theta
        a_r_z = np.array(
            [[math.cos(theta), -math.sin(theta), 0, 0],
             [math.sin(theta), math.cos(theta), 0, 0],
             [0, 0, 1, 0],
             [0, 0, 0, 1]])

        # Translation Transformation along z axis by d
        a_t_z = np.array(
            [[1, 0, 0, 0],
             [0, 1, 0, 0],
             [0, 0, 1, d],
             [0, 0, 0, 1]])

        # Translation Transformation along x axis by a
        a_t_x = np.array(
            [[1, 0, 0, a],
             [0, 1, 0, 0],
             [0, 0, 1, 0],
             [0, 0, 0, 1]])

        # Rotation Transformation about x axis by alpha
        a_r_x = np.array(
            [[1, 0, 0, 0],
             [0, math.cos(alpha), -math.sin(alpha), 0],
             [0, math.sin(alpha), math.cos(alpha), 0],
             [0, 0, 0, 1]])

        # For a transformation from frame {i} to frame {i-1}: transformed
        transformed = a_r_z @ a_t_z @ a_r_x @ a_t_x

        return transformed

    # Calculate end-effector position (x, y, z) using forward kinematics
    # Input:    joint angles in rads
    # Output:   end-effector position (x, y, z) expressed in base frame {0}
    def qarm_forward_kinematics(self, joint1, joint2, joint3, joint4):
        # Transformation matrices for all frames:
        # A{i-1}{i} = quanser_arm_dh(theta, d, a, alpha)

        A01 = self.qarm_dh(joint1, self._L1, 0, -math.pi/2)
        A12 = self.qarm_dh(joint2 - math.pi/2, 0, self._L2, 0)
        A23 = self.qarm_dh(joint3, 0, 0, -math.pi/2)
        A34 = self.qarm_dh(joint4, self._L3, 0, 0)

        A04 = A01 @ A12 @ A23 @ A34

        # Extract and return the x, y, z Position rounded to four decimal digits
        return round(A04[0, 3], 4), round(A04[1, 3], 4), round(A04[2, 3], 4)

    # Compute the position of the end-effector using inverse kinematics
    #
    # The solution is based on the geometric configuration of the QArm
    # where the upper links are contained within the vertical plane rotating
    # with the based joint angle q1.
    # The frame definition is consistent with the S&V DH convention.
    # Inputs: end-effector position, p_x, p_y, p_z
    # Outputs: joint angles in rads (base, shoulder, elbow) based on inverse kinematics
    def qarm_inverse_kinematics(self, p_x, p_y, p_z):

        # Initialization
        q_base = 0
        q_shoulder = 0
        q_elbow = 0

        # Base angle:
        q_base = math.atan2(p_y, p_x)

        # Geometric definitions
        # Radial distance (R) projection onto the horizontal plane
        R = math.sqrt(p_x**2 + p_y**2)

        # Vertical offset within the verical plane from Frame 1 to End-Effector
        # Note: Frame 1 y-axis points downward (negative global Z-axis direction)
        Z = self._L1 - p_z

        # Distance from Frame 1 to End-Effector Frame
        Lambda = math.sqrt(R**2 + Z**2)

        # Angle of Lambda vector from horizontal plane (Frame 1)
        # Note: theta is measured about z-axis of Frame 1 so positive theta
        # rotates Lambda "down".
        theta = math.atan2(Z, R)

        # Based angle of the triangle formed by L2, L3 and Lambda
        # Computed using cosine law
        # Note: The sign of alpha determines whether it is elbow up (alpha < 0) or
        # elbow down (alpha > 0) configuration (i.e., consistent with Frame 1)
        alpha = math.acos(-(self._L3**2 - self._L2**2 - Lambda**2) / (2*self._L2*Lambda))

        #Solve for q_shoulder; elbow up solution
        q_shoulder = math.pi/2 + (theta - alpha)

        #Solve for q_elbow, elbow up solution
        q_elbow = math.atan2(self._L2 - R*math.sin(q_shoulder) + Z*math.cos(q_shoulder), R*math.cos(q_shoulder) + Z*math.sin(q_shoulder))

        # Return the joint angles in degrees
        return q_base, q_shoulder, q_elbow
        
        
    def qarm_get_gripper_object_properties(self):
        modc = comm_modular_container()

        self._comms.postMail(modc.qarm_RequestGripperObjectProperties(self._dev_num))
        self._comms.deliver()  
        
        #Fetch until new QArm data is received from simulation
        processing = True
        while processing:
            time.sleep(0.01)
            count = self._comms.fetch()
            
            if (count > 0):
                # Get QArm containers
                QA_In = self._comms.checkMail(comm_modular_container.ID_QARM)
                       
                for QA_container in QA_In:
                    # Get just the gripper object container
                    if (QA_container.device_function == comm_modular_container.FCN_QARM_RESPONSE_GRIPPER_OBJECT_PROPERTIES): 
                        self.object_id, self.object_mass, self.object_properties = QA_container.qarm_ResponseGripperObjectProperties()
                        processing = False
        
        
        return self.object_id, self.object_mass, self.object_properties

        
    def ping(self):
        # Creat ping request packet
        modc = comm_modular_container()
        self._comms.postMail(modc.common_RequestPing(comm_modular_container.ID_QARM, 0))

        #Send all data back to QIL
        self._comms.deliver()





########################################################################
#################### VIRTUAL ROTARY TABLE CLASSES ###################### 
########################################################################

######################### ROTARY TABLE SIM #############################

class rotarytable_sim:

    # Define class-level variables 
    
    _comms = None
    _comc = comm_modular_container()
    _dev_num = 0
    
    _tof_value = None
    _encoder_value = None
    _load_cell_value = None
    
    _relative_x_short = None
    _relative_y_short = None
    _relative_z_short = None
    _properties_short = None

    _relative_x_tall = None
    _relative_y_tall = None
    _relative_z_tall = None
    _properties_tall = None

    
    # Initilize Virtual rotary table
    def __init__(self, postman, device_num = 0):
        self._comms = postman
        self._dev_num = device_num
        print ("Virtual rotary table initialized")
    
    ##--------------- VIRTUAL ROTARY TABLE METHODS ----------------------- 
    
    # Read all sensors: encoder and ToF
    def _read_all_sensors (self):
        modc = comm_modular_container()
        
        read_encoder = False
        read_tof = False
        read_prox_short = False
        read_prox_tall = False
        read_load_mass = False
        
        

        #Request sensor feedback; read values on next fetch
        self._comms.postMail(modc.srv02BottleTable_RequestEncoder(0))
        self._comms.postMail(modc.srv02BottleTable_RequestTOF(0))
        self._comms.postMail(modc.srv02BottleTable_RequestProximityShort(0))
        self._comms.postMail(modc.srv02BottleTable_RequestProximityTall(0))
        self._comms.postMail(modc.srv02BottleTable_RequestLoadMass(0))

        self._comms.deliver()
        time.sleep(0.01)
        
        #Fetch until all data is received from simulation
        while not(read_encoder and read_tof and read_prox_short and read_prox_tall and read_load_mass):
            time.sleep(0.01)
            count = self._comms.fetch()
            
            if (count > 0):
                #Extract rotary table containers
                TT_In = self._comms.checkMail(modc.ID_SRV02BOTTLETABLE)
                    
                ###Parse each container###
                for TT_container in TT_In:
                    if TT_container.device_function == comm_modular_container.FCN_SRV02BT_RESPONSE_ENCODER: 
                        self._encoder_value = TT_container.srv02BottleTable_ResponseEncoder()
                        read_encoder = True

                    elif TT_container.device_function == comm_modular_container.FCN_SRV02BT_RESPONSE_TOF:
                        self._tof_value = TT_container.srv02BottleTable_ResponseTOF()
                        read_tof = True
                    
                    elif (TT_container.device_function == comm_modular_container.FCN_SRV02BT_RESPONSE_PROXIMITY_SHORT):
                        self._relative_x_short, self._relative_y_short, self._relative_z_short, self._properties_short = TT_container.srv02BottleTable_ResponseProximityShort()
                        read_prox_short = True
     
                    elif (TT_container.device_function == comm_modular_container.FCN_SRV02BT_RESPONSE_PROXIMITY_TALL):
                        self._relative_x_tall, self._relative_y_tall, self._relative_z_tall, self._properties_tall = TT_container.srv02BottleTable_ResponseProximityTall()
                        read_prox_tall = True

                    elif (TT_container.device_function == comm_modular_container.FCN_SRV02BT_RESPONSE_LOAD_MASS):
                        self._load_cell_value = TT_container.srv02BottleTable_ResponesLoadMass()
                        read_load_mass = True

    # Return encoder value
    def read_encoder (self):
        self._read_all_sensors ()
        return self._encoder_value

    # Return ToF value
    def read_tof_sensor (self):
        self._read_all_sensors ()
        return int(self._tof_value)
    
    # Return short proximity sensor values
    def read_proximity_sensor_short (self):
        self._read_all_sensors ()
        return self._relative_x_short, self._relative_y_short, self._relative_z_short, self._properties_short

    # Return tall proximity sensor values
    def read_proximity_sensor_tall (self):
        self._read_all_sensors ()
        return self._relative_x_tall, self._relative_y_tall, self._relative_z_tall, self._properties_tall
        
    def read_load_cell (self):
        self._read_all_sensors()
        return int(self._load_cell_value)        

    # Rotate table for a given speed (positive goes CW; negative goes CCW)
    def _rotate (self, speed):
        modc = comm_modular_container()

        self._comms.postMail(modc.srv02BottleTable_CommandSpeed(0, speed))

        #Send all data back to QIL
        self._comms.deliver()
        time.sleep(0.1)

    # Rotate table clockwise for a given positive speed
    def rotate_clockwise (self, speed):
        modc = comm_modular_container()

        self._comms.postMail(modc.srv02BottleTable_CommandSpeed(0, speed))

        #Send all data back to QIL
        self._comms.deliver()
        time.sleep(0.1)
    
    # Rotate table counter clockwise for a positive speed
    def rotate_counterclockwise (self, speed):
        modc = comm_modular_container()

        self._comms.postMail(modc.srv02BottleTable_CommandSpeed(0, -speed))

        #Send all data back to QIL
        self._comms.deliver()
        time.sleep(0.1)
    
    # Rotate table for given angle in degrees in CW direction (open-loop)
    def command_rel_position_cw(self, angle, speed = 0.1):
        # Encoder counts to degrees
        K_enc = 360/4096
        
        initial_encoder_count = self.read_encoder()
        #print("Init encoder: ", initial_encoder_count)
        
        current_encoder_count = initial_encoder_count
        
        while (current_encoder_count - initial_encoder_count)*K_enc < angle:
            current_encoder_count = self.read_encoder()
            #print("Curr encoder: ", current_encoder_count)
            self.rotate_clockwise(speed)
        
        #print ("Commanded (deg): ", angle)
        #print ("Actual (deg): ", (current_encoder_count - initial_encoder_count)*K_enc)
        self.stop_table()
        
    # Rotate table for given angle in degrees (closed-loop; proportional-only for now)
    # Both positive and negative angle can be commanded. However, do no rotate table
    # CCW past initial zero position. Encoder wraps.
    def command_rel_position_pid(self, angle, max_voltage = 2, tolerance = 0.05):
        # Encoder counts to degrees
        K_enc = 360/4096
        #Proportional gain
        Kp = .02
        # Saturation voltage
        saturation_voltage = max_voltage
        
        initial_encoder_count = self.read_encoder()
        #print("Init encoder: ", initial_encoder_count )
        
        current_encoder_count = initial_encoder_count
        
        current_angle = 0
        
        if angle > 0: 
            error = angle - current_angle
        else:
            error = current_angle - angle
        
        while error > tolerance:
            PTerm = Kp*error
            speed = PTerm
            
            if angle > 0: 
                direction = 1
            else:
                direction = -1
                
            # Saturate speed at saturation value
            if speed > saturation_voltage:
                speed = saturation_voltage
            
            self._rotate(direction*speed)
            current_encoder_count = self.read_encoder()
            current_angle = (current_encoder_count - initial_encoder_count)*K_enc
            
            if angle > 0:
                error = angle - current_angle
            else:
                error = current_angle - angle
        
            #print ("Commanded (deg): ", angle)
            #print ("Actual (deg): ", (current_encoder_count - initial_encoder_count)*K_enc)
        self.stop_table()

    # Stop rotarytable
    def stop_table (self):
        modc = comm_modular_container()
    
        speed = 0.0
        self._comms.postMail(modc.srv02BottleTable_CommandSpeed(0, speed))

        #Send all data back to QIL
        self._comms.deliver()
        time.sleep(0.1)
    
    # Spawn single bottle; color is an array of 3 elements of [r, g, b]; element values from 0-1;
    # material is a custom text, set to metal to spawn soda can, else plasic bottle will be spawned
    def spawn_single_bottle (self, color, mass, material):
        modc = comm_modular_container()
        color_r = color[0]
        color_g = color[1] 
        color_b = color[2]

        metallic_roughness = 0.3
        
        if material == "metal":
            metallic = 1
        else:
            metallic = 0
            
        #device_num, height, diameter, metallic, color_r, color_g, color_b, color_a, roughness, mass, properties
        self._comms.postMail(modc.srv02BottleTable_SpawnContainer(0, 0.1, 0.65, metallic, color_r, color_g, color_b, 1, metallic_roughness, mass, material))
        
        #Send all data back to QIL
        self._comms.deliver()
        time.sleep(0.1)
 
    def ping(self):
        # Creat ping request packet
        modc = comm_modular_container()
        self._comms.postMail(modc.common_RequestPing(comm_modular_container.ID_SRV02BOTTLETABLE, 0))

        #Send all data back to QIL
        self._comms.deliver()
        
        
        
########################################################################
######################## VIRTUAL EMG CLASSES ########################### 
########################################################################

######################### EMG SIM #############################

class EMG_sim:

    # Define class-level variables 
    
    _comms = None
    _comc = comm_modular_container()
    _dev_num = 0
    
    _emg_left = 0.0
    _emg_right = 0.0
    
    
    # Initilize EMG sensor
    def __init__(self, postman, device_num = 0):
        self._comms = postman
        self._dev_num = device_num
        print ("Virtual EMG initialized")
    
    ##--------------- VIRTUAL EMG METHODS ----------------------- 
    
    # Read all sensors
    def read_all_sensors (self):
        modc = comm_modular_container()

        #Request sensor feedback; read values on next fetch
        self._comms.postMail(modc.EMG_RequestState(0))
        self._comms.deliver()
        time.sleep(0.1)
        
        #Extract emg containers
        TT_In = self._comms.checkMail(modc.ID_EMG_INTERFACE)
        
        
        while(len(TT_In) == 0): 
            #Fetch until new data is received from simulation
            count = self._comms.fetch()
            time.sleep(0.01)

            #Extract emg containers
            TT_In = self._comms.checkMail(modc.ID_EMG_INTERFACE)  
            
        ###Parse each container###
        if len(TT_In) > 0:
            for TT_container in TT_In:
                if TT_container.device_function == comm_modular_container.FCN_EMG_RESPONSE_STATE: 
                    self._emg_left, self._emg_right = TT_container.EMG_ResponseState()
                    
        return self._emg_left, self._emg_right

 
    def ping(self):
        # Creat ping request packet
        modc = comm_modular_container()
        self._comms.postMail(modc.common_RequestPing(comm_modular_container.ID_EMG_INTERFACE, 0))

        #Send all data back to QIL
        self._comms.deliver()
                
########################################################################
####################### GENERIC SPAWNER CLASS ########################## 
########################################################################

class genericSpawn_sim:

    # Define class-level variables 
    
    _comms = None
    _comc = comm_modular_container()
    _dev_num = 0
    
    _emg_left = 0.0
    _emg_right = 0.0
    
    
    # Initilize EMG sensor
    def __init__(self, postman, device_num = 0):
        self._comms = postman
        self._dev_num = device_num
        print ("Generic Spawner device {} initialized".format(device_num))
        
        
    def spawn(self, spawn_type):
        modc = comm_modular_container()

        self._comms.postMail(modc.genericSpawner_Spawn(self._dev_num, spawn_type))
        self._comms.deliver()
        time.sleep(0.01)
        success = self._wait_for_spawn_ack()
       
        return success
        
    def _wait_for_spawn_ack (self):
        
        modc = comm_modular_container()
        success = False

        #Extract  containers
        TT_In = self._comms.checkMail(modc.ID_GENERIC_SPAWNER)
                
        while(len(TT_In) == 0): 
            #Fetch until new data is received from simulation
            count = 0
            while count == 0:
                count = self._comms.fetch()
                time.sleep(0.01)

            #Extract emg containers
            TT_In = self._comms.checkMail(modc.ID_GENERIC_SPAWNER)  
            
        ###Parse each container###
        if len(TT_In) > 0:
            for TT_container in TT_In:
                if TT_container.device_function == comm_modular_container.FCN_GENERIC_SPAWNER_SPAWN_ACK: 
                    success = TT_container.genericSpawner_SpawnAck()
                    
        return success
        
    def spawn_with_properties(self, spawn_type, mass, properties_string):
        modc = comm_modular_container()

        self._comms.postMail(modc.genericSpawner_Spawn_with_Properties(self._dev_num, spawn_type, mass, properties_string))
        self._comms.deliver()
        time.sleep(0.01)
        success = self._wait_for_spawn_with_properties_ack()
       
        return success
        
    def _wait_for_spawn_with_properties_ack (self):
        
        modc = comm_modular_container()
        success = False

        #Extract  containers
        TT_In = self._comms.checkMail(modc.ID_GENERIC_SPAWNER)
                
        while(len(TT_In) == 0): 
            #Fetch until new data is received from simulation
            count = 0
            while count == 0:
                count = self._comms.fetch()
                time.sleep(0.01)

            #Extract emg containers
            TT_In = self._comms.checkMail(modc.ID_GENERIC_SPAWNER)  
            
        ###Parse each container###
        if len(TT_In) > 0:
            for TT_container in TT_In:
                if TT_container.device_function == comm_modular_container.FCN_GENERIC_SPAWNER_SPAWN_ACK: 
                    success = TT_container.genericSpawner_SpawnAck()
                    
        return success        
 
    def ping(self):
        # Creat ping request packet
        modc = comm_modular_container()
        self._comms.postMail(modc.common_RequestPing(comm_modular_container.ID_GENERIC_SPAWNER, 0))

        #Send all data back to QIL
        self._comms.deliver()
        
        
########################################################################
############################# AUTOCLAVE ################################ 
########################################################################

class autoclave_sim:

    # Define class-level variables 
    
    _comms = None
    _comc = comm_modular_container()
    _dev_num = 0
      
    
    # Initilize EMG sensor
    def __init__(self, postman, device_num = 0):
        self._comms = postman
        self._dev_num = device_num
        print ("Autoclave device {} initialized".format(device_num))
        
        
    def open_drawer(self, open_drawer):
        modc = comm_modular_container()

        self._comms.postMail(modc.autoclave_OpenDrawer(self._dev_num, open_drawer))
        self._comms.deliver()
        time.sleep(0.01)
        success = self._wait_for_ack()
       
        return
        
    def _wait_for_ack (self):
        
        modc = comm_modular_container()
        success = False

        #Extract  containers
        TT_In = self._comms.checkMail(modc.ID_AUTOCLAVE)
                
        while(len(TT_In) == 0): 
            #Fetch until new data is received from simulation
            count = 0
            while count == 0:
                count = self._comms.fetch()
                time.sleep(0.01)

            #Extract emg containers
            TT_In = self._comms.checkMail(modc.ID_AUTOCLAVE)  
   
        return 
 
    def ping(self):
        # Creat ping request packet
        modc = comm_modular_container()
        self._comms.postMail(modc.common_RequestPing(comm_modular_container.ID_GENERIC_SPAWNER, 0))

        #Send all data back to QIL
        self._comms.deliver()        
                
                
########################################################################
############################## SMARTBOX ################################ 
########################################################################

class smartbox_sim:

    # Define class-level variables 
    
    _comms = None
    _comc = comm_modular_container()
    _dev_num = 0
    _r = 1
    _g = 1
    _b = 1
    _metallic = 0
    _roughness = 0
      
    
    # Initilize EMG sensor
    def __init__(self, postman, device_num = 0):
        self._comms = postman
        self._dev_num = device_num
        #print ("Smartbox device {} initialized".format(device_num)) #Martha Removed this print statement
        
        

    def get_surface_properties (self):
        modc = comm_modular_container()

        #Request sensor feedback; read values on next fetch
        self._comms.postMail(modc.smartbox_Request_Surface_Properties(self._dev_num))
        self._comms.deliver()
        time.sleep(0.1)
        
        #Extract emg containers
        TT_In = self._comms.checkMail(modc.ID_SMARTBOX)
                
        while(len(TT_In) == 0): 
            #Fetch until new data is received from simulation
            count = self._comms.fetch()
            time.sleep(0.01)

            #Extract emg containers
            TT_In = self._comms.checkMail(modc.ID_SMARTBOX)  
            
        ###Parse each container###
        if len(TT_In) > 0:
            for TT_container in TT_In:
                if TT_container.device_function == comm_modular_container.FCN_SMARTBOX_RESPONSE_SURFACE_PROPERTIES: 
                    self._r, self._g, self._b, self._metallic, self._roughness = TT_container.smartbox_Response_Surface_Properties()
                    
        return self._r, self._g, self._b, self._metallic, self._roughness
        
    def get_position (self):
        modc = comm_modular_container()
        _world_xyz = [0,0,0]

        #Request sensor feedback; read values on next fetch
        self._comms.postMail(modc.common_RequestWorldTransform(comm_modular_container.ID_SMARTBOX ,self._dev_num))
        self._comms.deliver()
        time.sleep(0.1)
        
        #Extract emg containers
        TT_In = self._comms.checkMail(modc.ID_SMARTBOX)
                
        while(len(TT_In) == 0): 
            #Fetch until new data is received from simulation
            count = self._comms.fetch()
            time.sleep(0.01)

            #Extract emg containers
            TT_In = self._comms.checkMail(modc.ID_SMARTBOX)  
            
        ###Parse each container###
        if len(TT_In) > 0:
            for TT_container in TT_In:
                if TT_container.device_function == comm_modular_container.FCN_RESPONSE_WORLD_TRANSFORM: 
                    _world_xyz[0], _world_xyz[1], _world_xyz[2], rot_x, rot_y, rot_z, scale_x, scale_y, scale_z = TT_container.common_ResponseWorldTransform()
                    
        return _world_xyz
        
        
 
    def ping(self):
        # Creat ping request packet
        modc = comm_modular_container()
        self._comms.postMail(modc.common_RequestPing(comm_modular_container.ID_SMARTBOX, self._dev_num))

        #Send all data back to QIL
        self._comms.deliver()        
                                