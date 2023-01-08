# Using the DynamixelSDK
# Kyle DuFrene, January 2023, OSU Robotics

## Must run with Python 3!!

import pickle as pkl
from math import pi
import numpy as np
from dynamixel_sdk import *                    # Uses Dynamixel SDK library

class TwoVTwo:
    def __init__(self): 
        # 2v2 position variables   
        self.dy_0_pos = 210
        self.dy_0_limits = [412, 37] 
        self.dy_1_pos = 570
        self.dy_1_limits = [932, 247]
        self.dy_2_pos = 830
        self.dy_2_limits = [619, 1023]      
        self.dy_3_pos = 300
        self.dy_3_limits = [0, 657] 

        
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
        self.DXL0_ID                     = 0                 # Dynamixel#1 ID : 0
        self.DXL1_ID                     = 1                 # Dynamixel#1 ID : 1
        self.DXL2_ID                     = 2                 # Dynamixel#1 ID : 2
        self.DXL3_ID                     = 3                 # Dynamixel#1 ID : 3


        # Verify this!!
        self.DEVICENAME                  = '/dev/ttyUSB0'

        self.TORQUE_ENABLE               = 1                 # Value for enabling the torque
        self.TORQUE_DISABLE              = 0                 # Value for disabling the torque
        self.DXL_MOVING_STATUS_THRESHOLD = 1             # Dynamixel moving status threshold

        self.index = 0
        self.dxl0_goal_position = self.dy_0_pos
        self.dxl1_goal_position = self.dy_1_pos  
        self.dxl2_goal_position = self.dy_2_pos   
        self.dxl3_goal_position = self.dy_3_pos 
        self.dxl_led_value = [0x00, 0x01]      
        self.control_val_0 = 63
        self.control_val_1 = 63        
        self.control_val_2 = 63
        self.control_val_3 = 63                                         # Dynamixel LED value for write

        self.j1 = []
        self.j2 = []
        self.j3 = []
        self.j4 = []
        

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
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(self.BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            quit()
	

        # Enable Dynamixel#0 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL0_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % self.DXL0_ID)


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

        
        # Add parameter storage for Dynamixel#0 present position
        dxl_addparam_result = self.groupBulkRead.addParam(self.DXL0_ID, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupBulkRead addparam failed" % self.DXL0_ID)
            quit()

        # Add parameter storage for Dynamixel#1 present position
        dxl_addparam_result = self.groupBulkRead.addParam(self.DXL1_ID, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupBulkRead addparam failed" % self.DXL1_ID)
            quit()

        # Add parameter storage for Dynamixel#2 present position
        dxl_addparam_result = self.groupBulkRead.addParam(self.DXL2_ID, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupBulkRead addparam failed" % self.DXL2_ID)
            quit()

        # Add parameter storage for Dynamixel#3 present position
        dxl_addparam_result = self.groupBulkRead.addParam(self.DXL3_ID, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupBulkRead addparam failed" % self.DXL3_ID)
            quit()
    
    def get_pickle_data(self):
        with open("/home/kyle/Hands/src/Example_angles/angles_E.pkl", 'rb') as f:
            data = pkl.load(f)
        
        self.j0 = self.convert_rad_to_pos(data["joint_1"])
        self.j1 = self.convert_rad_to_pos(data["joint_2"])
        self.j2 = self.convert_rad_to_pos(data["joint_3"])
        self.j3 = self.convert_rad_to_pos(data["joint_4"])


    def convert_rad_to_pos(self, rad):
        # Take input pickle data and convert radians to 0 to 1023 position
        
        # XL-320 is 1023 to 300 degrees

        # Convert to degrees
        deg = np.multiply(rad, (180/pi))
        # .2932 degrees per step

        # Pos is deviation from center (0 degrees), defined in init
        pos = np.multiply(deg, (1023/300))
        pos  = pos.astype(int)

        return pos

    def map_pickle(self, i):
        # Set the positions in terms of actual calibrated motor positions
        self.dxl0_goal_position = self.dy_0_pos - self.j0[i]
        self.dxl1_goal_position = self.dy_1_pos - self.j1[i]
        self.dxl2_goal_position = self.dy_2_pos - self.j2[i]
        self.dxl3_goal_position = self.dy_3_pos - self.j3[i]

    
    def run(self):
        # Sends the goal positions to the Dynamixels

        # Allocate goal position value into byte array
        param_goal_position0 = [DXL_LOBYTE(DXL_LOWORD(self.dxl0_goal_position)), DXL_HIBYTE(DXL_LOWORD(self.dxl0_goal_position)), DXL_LOBYTE(DXL_HIWORD(self.dxl0_goal_position)), DXL_HIBYTE(DXL_HIWORD(self.dxl0_goal_position))]
        param_goal_position1 = [DXL_LOBYTE(DXL_LOWORD(self.dxl1_goal_position)), DXL_HIBYTE(DXL_LOWORD(self.dxl1_goal_position)), DXL_LOBYTE(DXL_HIWORD(self.dxl1_goal_position)), DXL_HIBYTE(DXL_HIWORD(self.dxl1_goal_position))]
        param_goal_position2 = [DXL_LOBYTE(DXL_LOWORD(self.dxl2_goal_position)), DXL_HIBYTE(DXL_LOWORD(self.dxl2_goal_position)), DXL_LOBYTE(DXL_HIWORD(self.dxl2_goal_position)), DXL_HIBYTE(DXL_HIWORD(self.dxl2_goal_position))]
        param_goal_position3 = [DXL_LOBYTE(DXL_LOWORD(self.dxl3_goal_position)), DXL_HIBYTE(DXL_LOWORD(self.dxl3_goal_position)), DXL_LOBYTE(DXL_HIWORD(self.dxl3_goal_position)), DXL_HIBYTE(DXL_HIWORD(self.dxl3_goal_position))]
        
        # Add Dynamixel#0 goal position value to the Bulkwrite parameter storage
        dxl_addparam_result = self.groupBulkWrite.addParam(self.DXL0_ID, self.ADDR_GOAL_POSITION, self.LEN_GOAL_POSITION, param_goal_position0)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupBulkWrite addparam failed" % self.DXL0_ID)
            quit()
        
        # Add Dynamixel#1 goal position value to the Bulkwrite parameter storage
        dxl_addparam_result = self.groupBulkWrite.addParam(self.DXL1_ID, self.ADDR_GOAL_POSITION, self.LEN_GOAL_POSITION, param_goal_position1)
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

        
        # Bulkwrite goal position and LED value
        dxl_comm_result = self.groupBulkWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        # Clear bulkwrite parameter storage
        self.groupBulkWrite.clearParam()




    
    def end_program(self):
        ###### This code runs upon exit ######

        # Clear bulkread parameter storage
        self.groupBulkRead.clearParam()

        # Disable Dynamixel#0 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL0_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
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

        #self.portHandler.closePort()   



if __name__== "__main__":
    hand2v2 = TwoVTwo()    
    
    try:
        while(True):
            hand2v2.get_pickle_data()
            
            for i in range(len(hand2v2.j1)):
                input("Press Enter to continue to next step. Step num: " + str(i) + "/" + str(len(hand2v2.j1)))
                hand2v2.map_pickle(i)
                print("0: " + str(hand2v2.dxl0_goal_position) + "  1: " + str(hand2v2.dxl1_goal_position) + "  2: " + str(hand2v2.dxl2_goal_position) + "  3: " + str(hand2v2.dxl3_goal_position))
                hand2v2.run()
    except KeyboardInterrupt:
        hand2v2.end_program()

    hand2v2.end_program()