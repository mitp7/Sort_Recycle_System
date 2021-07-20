import sys
import time
import math
sys.path.append('../')

from Common_Libraries.modular_comm import comm_modular_server
from Common_Libraries.modular_comm import comm_modular_container

class postman:
    
    #Container Variables
    _comsvr = None
    _inbox_OTHER = []
    _inbox_EMG = []
    _inbox_QBOT = []
    _inbox_QARM = []
    _inbox_TABLE = []
    _inbox_GENERIC_SPAWNER = []
    _inbox_AUTOCLAVE = []
    _inbox_SMARTBOX = []

    def __init__(self, port):
        print ("Initializing Comm Server")

        self._comsvr = comm_modular_server(port)

        print ("Comm Server Initialized")

    def fetch(self):
        container_count = 0
        if self._comsvr.receive_new_data():
            more_containers = True

            while more_containers:
                container_count += 1
                c_in, more_containers = self._comsvr.get_next_container()
                
                #print("Received container ID {}".format(c_in.device_id))
                
                ##self._inbox.append(c_in)
                if c_in.device_id == comm_modular_container.ID_QBOT:
                    self._inbox_QBOT.append(c_in)
                elif c_in.device_id == comm_modular_container.ID_QBOT_BOX:
                    self._inbox_QBOT.append(c_in)
                elif c_in.device_id == comm_modular_container.ID_QARM:
                    self._inbox_QARM.append(c_in)
                elif c_in.device_id == comm_modular_container.ID_SRV02BOTTLETABLE:
                    self._inbox_TABLE.append(c_in)
                elif c_in.device_id == comm_modular_container.ID_EMG_INTERFACE:
                    self._inbox_EMG.append(c_in)   
                elif c_in.device_id == comm_modular_container.ID_GENERIC_SPAWNER:
                    self._inbox_GENERIC_SPAWNER.append(c_in)     
                elif c_in.device_id == comm_modular_container.ID_AUTOCLAVE:
                    self._inbox_AUTOCLAVE.append(c_in)                      
                elif c_in.device_id == comm_modular_container.ID_SMARTBOX:
                    self._inbox_SMARTBOX.append(c_in)  
                else:
                    self._inbox_OTHER.append(c_in)          
                    while len(self._inbox_OTHER) > 10:
                        self._inbox_OTHER.pop(0)
                    

        return container_count

    def checkMail(self, deviceID, deviceNUM = 0):
        out = []
        
        #print("QBOT {}, QArm {}, Table {}, EMG {}, Spawner {}".format(len(self._inbox_QBOT), len(self._inbox_QARM), len(self._inbox_TABLE), len(self._inbox_EMG), len(self._inbox_GENERIC_SPAWNER)))
        if deviceID == comm_modular_container.ID_QBOT:
            out = self._inbox_QBOT
            self._inbox_QBOT = []
        elif deviceID == comm_modular_container.ID_QARM:
            out = self._inbox_QARM
            self._inbox_QARM = []
        elif deviceID == comm_modular_container.ID_SRV02BOTTLETABLE:
            out = self._inbox_TABLE
            self._inbox_TABLE = []
        elif deviceID == comm_modular_container.ID_EMG_INTERFACE:
            out = self._inbox_EMG
            self._inbox_EMG = []
        elif deviceID == comm_modular_container.ID_GENERIC_SPAWNER:
            out = self._inbox_GENERIC_SPAWNER
            self._inbox_GENERIC_SPAWNER = []    
        elif deviceID == comm_modular_container.ID_AUTOCLAVE:
            out = self._inbox_AUTOCLAVE
            self._inbox_AUTOCLAVE = []   
        elif deviceID == comm_modular_container.ID_SMARTBOX:
            out = self._inbox_SMARTBOX
            self._inbox_SMARTBOX = []              
        elif deviceID > 0:
            no_match = []
            for c_inb in self._inbox_OTHER:
                if c_inb.device_id == deviceID and c_inb.device_number == deviceNUM:
                    out.append(c_inb)
                else:
                    no_match.append(c_inb)
            self._inbox_OTHER = no_match
        else:
            out = self._inbox_OTHER
            self._inbox_OTHER = [] 
        return out
    
    def postMail(self, c_post):
        self._comsvr.queue_container(c_post)

    def expressMail(self, c_exp):
        self._comsvr.send_container(c_exp)

    def deliver(self):
        bytes_sent = self._comsvr.send_queue()
        return bytes_sent
    
    def flush(self):
        self.deliver()
        time.sleep(0.01)
        self.fetch()
        self._inbox_OTHER = []
        self._inbox_QBOT = []
        self._inbox_QARM = []
        self._inbox_EMG = []
        self._inbox_TABLE = []
        self._inbox_GENERIC_SPAWNER = []
        self._inbox_AUTOCLAVE = []
        self._inbox_SMARTBOX = []
    
    #stop postman
    def close(self):
        self.__exit__()

    #Exit Routine
    def __exit__(self):
        self._comsvr.send_queue()
        self._comsvr.close()

