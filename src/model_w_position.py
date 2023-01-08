# Adapted from the DynamixelSDK
# Kyle DuFrene, March 2022, OSU Robotics


import os
import sys, tty, termios
import pygame
import pygame.midi
import keyboard
import time

fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
def getch():
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

class Model_Q:
    def __init__(self): 
        self.move_to = False
        self.just_switched = False
        self.mode = True # True is main, false is individual control

        # Model Q position variables   
        self.dy_4_pos = 0
        self.dy_4_limits = [960, 550] 
        self.dy_1_pos = 0
        self.dy_1_limits = [940, 540]
        self.dy_2_pos = 0 #rotation motor

        self.dy_2_limits = [290, 825]      
        self.dy_3_pos = 0
        self.dy_3_limits = [850, 430] 
        self.dy_5_pos = 0

        
        pygame.midi.init()
        self.ADDR_TORQUE_ENABLE          = 24
        self.ADDR_LED_RED                = 65
        self.LEN_LED_RED                 = 1         # Data Byte Length
        self.ADDR_GOAL_POSITION          = 30
        self.LEN_GOAL_POSITION           = 4         # Data Byte Length
        self.ADDR_PRESENT_POSITION       = 37
        self.LEN_PRESENT_POSITION        = 4         # Data Byte Length
        self.DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
        self.DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual
        self.BAUDRATE                    = 57600

        # DYNAMIXEL Protocol Version (1.0 / 2.0)
        # https://emanual.robotis.com/docs/en/dxl/protocol2/
        self.PROTOCOL_VERSION            = 2.0

        # Make sure that each DYNAMIXEL ID should have unique ID.
        self.DXL4_ID                     = 4                 # Dynamixel#1 ID : 0
        self.DXL1_ID                     = 1                 # Dynamixel#1 ID : 1
        self.DXL2_ID                     = 2                 # Dynamixel#1 ID : 2
        self.DXL3_ID                     = 3                 # Dynamixel#1 ID : 3
        self.DXL5_ID                     = 5                 # Dynamixel#1 ID : 5
        

        # Verify this!!
        self.DEVICENAME                  = '/dev/ttyUSB0'

        self.TORQUE_ENABLE               = 1                 # Value for enabling the torque
        self.TORQUE_DISABLE              = 0                 # Value for disabling the torque
        self.DXL_MOVING_STATUS_THRESHOLD = 1             # Dynamixel moving status threshold

        self.index = 0
        self.dxl4_goal_position = int(self.dy_4_limits[0])   #1000 released, 100 closed
        self.dxl_goal_position = self.dy_1_limits[0]  #1000 released, 100 closed
        self.dxl2_goal_position = self.dy_2_limits[0]   #rotate limits
        self.dxl3_goal_position = self.dy_3_limits[0]  #550 released, 1500 closed
        self.dxl5_goal_position = self.dy_5_limits[0]  #550 released, 1500 closed
        self.dxl6_goal_position = self.dy_6_limits[0]  #550 released, 1500 closed
        self.dxl_led_value = [0x00, 0x01]      
        self.control_val_1 = 63
        self.control_val_2 = 63        
        self.control_val_3 = 63
        self.control_val_4 = 63 
        self.control_val_5 = 63
        self.control_val_6 = 63                                           # Dynamixel LED value for write

        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(self.DEVICENAME)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        # Initialize GroupBulkWrite instance
        self.groupBulkWrite = GroupBulkWrite(self.portHandler, self.packetHandler)

        # Initialize GroupBulkRead instace for Present Position
        self.groupBulkRead = GroupBulkRead(self.portHandler, self.packetHandler)

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(self.BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()
	
	    
        devices = pygame.midi.get_count()

        if devices<1:
            print("No MIDI devices detected")
            exit(-1)
        print("Found %d MIDI devices" % devices)
		
        id = 3 # 3 for linux, 1 for windows

        if id is not None:
            input_dev = id
        else:
            input_dev = pygame.midi.get_default_input_id()
            if input_dev==-1:
                print("No default MIDI input device")
                exit(-1)
        print("Using input device %d" % input_dev)
		
        self.controller = pygame.midi.Input(input_dev)

        # Enable Dynamixel#4 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % self.DXL4_ID)


        # Enable Dynamixel#1 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % self.DXL1_ID)

        # Enable Dynamixel#2 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % self.DXL2_ID)

        # Enable Dynamixel#3 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % self.DXL3_ID)

        # Enable Dynamixel#5 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL5_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % self.DXL5_ID)

        # Enable Dynamixel#6 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL6_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % self.DXL6_ID)
        
        # Add parameter storage for Dynamixel#4 present position
        dxl_addparam_result = self.groupBulkRead.addParam(self.DXL4_ID, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupBulkRead addparam failed" % self.DXL4_ID)
            quit()

        # Add parameter storage for Dynamixel#1 present position
        dxl_addparam_result = self.groupBulkRead.addParam(self.DXL1_ID, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupBulkRead addparam failed" % self.DXL1_ID)
            quit()

        # Add parameter storage for Dynamixel#1 present position
        dxl_addparam_result = self.groupBulkRead.addParam(self.DXL2_ID, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupBulkRead addparam failed" % self.DXL2_ID)
            quit()

        # Add parameter storage for Dynamixel#3 present position
        dxl_addparam_result = self.groupBulkRead.addParam(self.DXL3_ID, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupBulkRead addparam failed" % self.DXL3_ID)
            quit()
        
        # Add parameter storage for Dynamixel#5 present position
        dxl_addparam_result = self.groupBulkRead.addParam(self.DXL5_ID, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupBulkRead addparam failed" % self.DXL5_ID)
            quit()

        # Add parameter storage for Dynamixel#6 present position
        dxl_addparam_result = self.groupBulkRead.addParam(self.DXL6_ID, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupBulkRead addparam failed" % self.DXL6_ID)
            quit()

    def model_q_run(self):

        # Allocate goal position value into byte array
        param_goal_position4 = [DXL_LOBYTE(DXL_LOWORD(self.dxl4_goal_position)), DXL_HIBYTE(DXL_LOWORD(self.dxl4_goal_position)), DXL_LOBYTE(DXL_HIWORD(self.dxl4_goal_position)), DXL_HIBYTE(DXL_HIWORD(self.dxl4_goal_position))]
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(self.dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(self.dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(self.dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(self.dxl_goal_position))]
        param_goal_position2 = [DXL_LOBYTE(DXL_LOWORD(self.dxl2_goal_position)), DXL_HIBYTE(DXL_LOWORD(self.dxl2_goal_position)), DXL_LOBYTE(DXL_HIWORD(self.dxl2_goal_position)), DXL_HIBYTE(DXL_HIWORD(self.dxl2_goal_position))]
        param_goal_position3 = [DXL_LOBYTE(DXL_LOWORD(self.dxl3_goal_position)), DXL_HIBYTE(DXL_LOWORD(self.dxl3_goal_position)), DXL_LOBYTE(DXL_HIWORD(self.dxl3_goal_position)), DXL_HIBYTE(DXL_HIWORD(self.dxl3_goal_position))]
        param_goal_position5 = [DXL_LOBYTE(DXL_LOWORD(self.dxl5_goal_position)), DXL_HIBYTE(DXL_LOWORD(self.dxl5_goal_position)), DXL_LOBYTE(DXL_HIWORD(self.dxl5_goal_position)), DXL_HIBYTE(DXL_HIWORD(self.dxl5_goal_position))]
        param_goal_position6 = [DXL_LOBYTE(DXL_LOWORD(self.dxl6_goal_position)), DXL_HIBYTE(DXL_LOWORD(self.dxl6_goal_position)), DXL_LOBYTE(DXL_HIWORD(self.dxl6_goal_position)), DXL_HIBYTE(DXL_HIWORD(self.dxl6_goal_position))]
        
        # Add Dynamixel#0 goal position value to the Bulkwrite parameter storage
        dxl_addparam_result = self.groupBulkWrite.addParam(self.DXL4_ID, self.ADDR_GOAL_POSITION, self.LEN_GOAL_POSITION, param_goal_position4)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupBulkWrite addparam failed" % self.DXL4_ID)
            quit()
        
        # Add Dynamixel#1 goal position value to the Bulkwrite parameter storage
        dxl_addparam_result = self.groupBulkWrite.addParam(self.DXL1_ID, self.ADDR_GOAL_POSITION, self.LEN_GOAL_POSITION, param_goal_position)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupBulkWrite addparam failed" % self.DXL1_ID)
            quit()

        # Add Dynamixel#2 goal position value to the Bulkwrite parameter storage
        dxl_addparam_result = self.groupBulkWrite.addParam(self.DXL2_ID, self.ADDR_GOAL_POSITION, self.LEN_GOAL_POSITION, param_goal_position2)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupBulkWrite addparam failed" % self.DXL2_ID)
            quit()

        # Add Dynamixel#3 goal position value to the Bulkwrite parameter storage
        dxl_addparam_result = self.groupBulkWrite.addParam(self.DXL3_ID, self.ADDR_GOAL_POSITION, self.LEN_GOAL_POSITION, param_goal_position3)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupBulkWrite addparam failed" % self.DXL3_ID)
            quit()

        # Add Dynamixel#5 goal position value to the Bulkwrite parameter storage
        dxl_addparam_result = self.groupBulkWrite.addParam(self.DXL5_ID, self.ADDR_GOAL_POSITION, self.LEN_GOAL_POSITION, param_goal_position5)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupBulkWrite addparam failed" % self.DXL5_ID)
            quit()
        
        # Add Dynamixel#6 goal position value to the Bulkwrite parameter storage
        dxl_addparam_result = self.groupBulkWrite.addParam(self.DXL6_ID, self.ADDR_GOAL_POSITION, self.LEN_GOAL_POSITION, param_goal_position6)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupBulkWrite addparam failed" % self.DXL6_ID)
            quit()


        
        # Bulkwrite goal position and LED value
        dxl_comm_result = self.groupBulkWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        # Clear bulkwrite parameter storage
        self.groupBulkWrite.clearParam()

        """ while 1:
            # Bulkread present position and LED status
            dxl_comm_result = groupBulkRead.txRxPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        
            # Check if groupbulkread data of Dynamixel#0 is available
            dxl_getdata_result = groupBulkRead.isAvailable(DXL0_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupBulkRead getdata failed" % DXL0_ID)
                quit()
        
            # Check if groupbulkread data of Dynamixel#1 is available
            dxl_getdata_result = groupBulkRead.isAvailable(DXL1_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupBulkRead getdata failed" % DXL1_ID)
                quit()

            # Check if groupbulkread data of Dynamixel#2 is available
            dxl_getdata_result = groupBulkRead.isAvailable(DXL2_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupBulkRead getdata failed" % DXL2_ID)
                quit()

            # Check if groupbulkread data of Dynamixel#3 is available
            dxl_getdata_result = groupBulkRead.isAvailable(DXL3_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupBulkRead getdata failed" % DXL3_ID)
                quit()


            # Get present position value
            dxl0_present_position = groupBulkRead.getData(DXL0_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            
            # Get present position value
            dxl1_present_position = groupBulkRead.getData(DXL1_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

            # Get present position value
            dxl2_present_position = groupBulkRead.getData(DXL2_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

            # Get present position value
            dxl3_present_position = groupBulkRead.getData(DXL3_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

            # Get LED value
            dxl2_led_value_read = groupBulkRead.getData(DXL2_ID, ADDR_LED_RED, LEN_LED_RED)

            print("[ID:%03d] Present Position : %d \t [ID:%03d] Present Position : %d \t [ID:%03d] Present Position : %d \t [ID:%03d] Present Position : %d" % (DXL0_ID, dxl0_present_position, DXL1_ID, dxl1_present_position, DXL2_ID, dxl2_present_position, DXL3_ID, dxl3_present_position))

            if (not (abs(dxl_goal_position[self.index] - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD)) and (not (abs(dxl3_goal_position[self.index] - dxl3_present_position) > DXL_MOVING_STATUS_THRESHOLD)) and (not (abs(dxl2_goal_position[self.index] - dxl2_present_position) > DXL_MOVING_STATUS_THRESHOLD)) and (not (abs(dxl_goal_position[self.index] - dxl0_present_position) > DXL_MOVING_STATUS_THRESHOLD)):
                break"""

        # Change goal position
        """if self.index == 0:
            self.index = 1
        else:
            self.index = 0"""

    def _poll(self):
        # keyboard
        """if keyboard.is_pressed("p"):
            self.index = 1
        else:
            self.index = 0"""

        #test

        # Check nano control
        # If we are recieving multiple inputs from the nanokontrol, take 4 at once
        test_data = self.controller.read(5)

        #print(test_data,type(test_data))
        
        if test_data:
            for x in test_data:
                #print(test_data[-1])
                #print(x)
                data=[x]
                for event in data:
                    #print(self.mode)
                    control = event[0]
                    if (control[0] & 0xF0) == 176:
                        control_id = control[1] | ((control[0] & 0x0F) << 8)
                        control_val = control[2]
                        if (self.mode):
                            # if in main control mode
                            if (control_id==46):
                                # Switch modes
                                self.just_switched = not self.just_switched
                                if (not self.just_switched):
                                    self.mode = not self.mode
                            elif (control_id==0):
                                self.linear_map_val(control_val,6)
                                self.control_val_6 = control_val
                            elif (control_id==1):
                                self.linear_map_val(control_val,5)
                                self.control_val_5 = control_val
                            elif (control_id==2):
                                self.linear_map_val(control_val,4)
                                self.control_val_4 = control_val
                            elif (control_id==3):
                                self.linear_map_val(control_val,2)
                                self.control_val_2 = control_val
                            elif (control_id==4):
                                self.linear_map_val(control_val,3)
                                self.control_val_3 = control_val
                            elif (control_id==5):
                                self.linear_map_val(control_val,1)
                                self.control_val_1 = control_val
        
                        else:
                            # if in individual control mode
                            if (control_id==46):
                                # Switch modes
                                self.just_switched = not self.just_switched
                                if (not self.just_switched):
                                    self.mode = not self.mode
                            elif (control_id==5):
                                self.map_val(control_val,1)
                            elif (control_id==4):
                                self.map_val(control_val,3)
                            elif (control_id==3):
                                self.map_val(control_val,2)
                            elif (control_id==2):
                               self.map_val(control_val,4)
                            elif (control_id==1):
                               self.map_val(control_val,5)
                            elif (control_id==0):
                               self.map_val(control_val,6)

        if (self.mode):
            self.linear_map_val(self.control_val_1,1)
            self.linear_map_val(self.control_val_2,2)
            self.linear_map_val(self.control_val_3,3)
            self.linear_map_val(self.control_val_4,4)
            self.linear_map_val(self.control_val_5,5)
            self.linear_map_val(self.control_val_6,6)



    
    def end_program(self):
        ###### This code runs upon exit ######

        # Clear bulkread parameter storage
        self.groupBulkRead.clearParam()

        # Disable Dynamixel#0 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        # Disable Dynamixel#1 Torque        # Add Dynamixel#3 goal position value to the Bulkwrite parameter storage
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        # Disable Dynamixel#2 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        # Disable Dynamixel#3 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        # Disable Dynamixel#5 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL5_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        # Disable Dynamixel#3 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL6_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        # Close port
        #self.portHandler.closePort()   

    def map_val(self, input_val, motor_id):
        self.move_to = True
        if (motor_id==4):
            # scale to match 0 motor
            self.dxl4_goal_position = int(self.dy_4_limits[0]+(input_val/127.0)*(self.dy_4_limits[1]-self.dy_4_limits[0]))
            #print(self.dxl4_goal_position)
            # lower position limit + blank/127 * (upper-lower)
        elif (motor_id==1):
            #scale to 1
            self.dxl_goal_position = int(self.dy_1_limits[0]+(input_val/127.0)*(self.dy_1_limits[1]-self.dy_1_limits[0]))
            #print(self.dxl_goal_position)
        elif (motor_id==2):
            #scale to 2
            self.dxl2_goal_position = int(self.dy_2_limits[0]+(input_val/127.0)*(self.dy_2_limits[1]-self.dy_2_limits[0]))
            #print(self.dxl2_goal_position)
        elif (motor_id==5):
            #scale to 2
            self.dxl5_goal_position = int(self.dy_5_limits[0]+(input_val/127.0)*(self.dy_5_limits[1]-self.dy_5_limits[0]))
            #print(self.dxl5_goal_position)
        elif (motor_id==6):
            #scale to 2
            self.dxl6_goal_position = int(self.dy_6_limits[0]+(input_val/127.0)*(self.dy_6_limits[1]-self.dy_6_limits[0]))
            #print(self.dxl2_goal_position)
        else:
            #scale to 3
            self.dxl3_goal_position = int(self.dy_3_limits[0]+(input_val/127.0)*(self.dy_3_limits[1]-self.dy_3_limits[0]))
            #print(self.dxl3_goal_position)


    def linear_map_val(self, input_val, motor_id):
        self.move_to = True
        #print('motor: ',motor_id,' input: ',input_val)
        if (input_val < 50):
            if (motor_id==1):
                self.dxl_goal_position = self.dxl_goal_position + (50-input_val)/5
                if (self.dxl_goal_position>self.dy_1_limits[0]):
                    self.dxl_goal_position = self.dy_1_limits[0]
            elif (motor_id==2):
                self.dxl2_goal_position = self.dxl2_goal_position - (50-input_val)/5
                if (self.dxl2_goal_position<self.dy_2_limits[0]):
                    self.dxl2_goal_position = self.dy_2_limits[0]
            elif (motor_id==3):
                self.dxl3_goal_position = self.dxl3_goal_position + (50-input_val)/5
                if (self.dxl3_goal_position>self.dy_3_limits[0]):
                    self.dxl3_goal_position = self.dy_3_limits[0]
            elif (motor_id==4):
                self.dxl4_goal_position = self.dxl4_goal_position + (50-input_val)/5
                if (self.dxl4_goal_position>self.dy_4_limits[0]):
                    self.dxl4_goal_position = self.dy_4_limits[0]
            elif (motor_id==5):
                self.dxl5_goal_position = self.dxl5_goal_position + (50-input_val)/5
                if (self.dxl5_goal_position>self.dy_5_limits[0]):
                    self.dxl5_goal_position = self.dy_5_limits[0]
            elif (motor_id==6):
                self.dxl6_goal_position = self.dxl6_goal_position - (50-input_val)/5
                if (self.dxl6_goal_position<self.dy_6_limits[0]):
                    self.dxl6_goal_position = self.dy_6_limits[0]
        elif (input_val > 77):
            if (motor_id==1):
                self.dxl_goal_position = self.dxl_goal_position - (input_val-77)/5
                if (self.dxl_goal_position<self.dy_1_limits[1]):
                    self.dxl_goal_position = self.dy_1_limits[1]
            elif (motor_id==2):
                self.dxl2_goal_position = self.dxl2_goal_position + (input_val-77)/5
                if (self.dxl2_goal_position>self.dy_2_limits[1]):
                    self.dxl2_goal_position = self.dy_2_limits[1]
            elif (motor_id==3):
                self.dxl3_goal_position = self.dxl3_goal_position - (input_val-77)/5
                if (self.dxl3_goal_position<self.dy_3_limits[1]):
                    self.dxl3_goal_position = self.dy_3_limits[1]
            elif (motor_id==4):
                self.dxl4_goal_position = self.dxl4_goal_position - (input_val-77)/5
                if (self.dxl4_goal_position<self.dy_4_limits[1]):
                    self.dxl4_goal_position = self.dy_4_limits[1]
            elif (motor_id==5):
                self.dxl5_goal_position = self.dxl5_goal_position - (input_val-77)/5
                if (self.dxl5_goal_position<self.dy_5_limits[1]):
                    self.dxl5_goal_position = self.dy_5_limits[1]
            elif (motor_id==6):
                self.dxl6_goal_position = self.dxl6_goal_position + (input_val-77)/5
                if (self.dxl6_goal_position>self.dy_6_limits[1]):
                    self.dxl6_goal_position = self.dy_6_limits[1]
            # do not
        #if (motor_id==0):
        #print('input: ', self.dxl0_goal_position)

if __name__== "__main__":
    obj_model_q = Model_Q()
    try:
        while(True):
            obj_model_q._poll()
            if (obj_model_q.move_to == True):
                obj_model_q.model_q_run()
                obj_model_q.move_to = False
    except KeyboardInterrupt:
        print('PortMidi: Bad Pointer is a known error, just ignore it :(')
        obj_model_q.end_program()
    obj_model_q.end_program()