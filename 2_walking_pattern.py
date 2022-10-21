from dynamixel_sdk import *                    # Uses Dynamixel SDK library
import time  
import math 

MY_DXL = 'P_SERIES'     # PH54, PH42, PM54

angle_1 = 30
angle_2 = 30
angle_3 = 30

angle_4 = 30
angle_5 = 30
angle_6 = 30


coef = 2789             # conversion to encoder
# Control table address

if MY_DXL == 'P_SERIES' or MY_DXL == 'PRO_A_SERIES':
    ADDR_TORQUE_ENABLE          = 512        # Control table address is different in DYNAMIXEL model
    ADDR_GOAL_POSITION          = 564
    LEN_GOAL_POSITION           = 4          # Data Byte Length
    ADDR_PRESENT_POSITION       = 580
    LEN_PRESENT_POSITION        = 4          # Data Byte Length
    DXL_MINIMUM_POSITION_VALUE  = 0     # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = angle_1 * coef     # Refer to the Maximum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE_2 = angle_2 * coef
    DXL_MAXIMUM_POSITION_VALUE_3 = angle_3 * coef
    DXL_MAXIMUM_POSITION_VALUE_4 = angle_4 * coef
    DXL_MAXIMUM_POSITION_VALUE_5 = angle_5 * coef
    DXL_MAXIMUM_POSITION_VALUE_6 = angle_6 * coef
    
    BAUDRATE                    = 1000000

# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION            = 2.0

# Make sure that each DYNAMIXEL ID should have unique ID.
#DXL_ID = [1, 2, 5]
# ldxl = len(DXL_ID)

#D = {1:30, 2:90, 5:10}   # dictionary: key - dynamixel ID; value - angle in degree




DXL1_ID                     = 0                 # Dynamixel#1 ID : 1
DXL2_ID                     = 1                 # Dynamixel#1 ID : 2
DXL3_ID                     = 2                 # Dynamixel#1 ID : 2
DXL4_ID                     = 3                 # Dynamixel#1 ID : 2
DXL5_ID                     = 4                 # Dynamixel#1 ID : 2
DXL6_ID                     = 5                 # Dynamixel#1 ID : 2

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = '/dev/ttyUSB0'

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

index = 0
index_2 = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position
dxl_goal_position_2 = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE_2]         # Goal position
dxl_goal_position_3 = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE_3]
dxl_goal_position_4 = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE_4]
dxl_goal_position_5 = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE_5]
dxl_goal_position_6 = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE_6]

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler_2 = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupSyncWrite instance
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

# Initialize GroupSyncWrite instance2
groupSyncWrite_2 = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

# Initialize GroupSyncRead instace for Present Position
groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

# Initialize GroupSyncRead_2 instace for Present Position
groupSyncRead_2 = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

# def check():
#     cnt = 0
#     for x in range(ldxl):
#         dxl1_present_position = groupSyncRead.getData(DXL_ID[x], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
#         #if not (abs(dxl_goal_position[index] - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD)  (abs(dxl_goal_position_2[index] - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD):
#         cnt += 1
#     if cnt == ldxl:
#         return True
#     else:
#         return False


    

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

# Enable Dynamixel#1 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL1_ID)

# Enable Dynamixel#2 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL2_ID)
    
    # Enable Dynamixel#3 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL3_ID)
    
# Enable Dynamixel#4 Torque
dxl_comm_result_2, dxl_error = packetHandler_2.write1ByteTxRx(portHandler, DXL4_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result_2 != COMM_SUCCESS:
    print("%s" % packetHandler_2.getTxRxResult(dxl_comm_result_2))
elif dxl_error != 0:
    print("%s" % packetHandler_2.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL4_ID)

# Enable Dynamixel#5 Torque
dxl_comm_result_2, dxl_error = packetHandler_2.write1ByteTxRx(portHandler, DXL5_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result_2 != COMM_SUCCESS:
    print("%s" % packetHandler_2.getTxRxResult(dxl_comm_result_2))
elif dxl_error != 0:
    print("%s" % packetHandler_2.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL5_ID)
    
# Enable Dynamixel#6 Torque
dxl_comm_result_2, dxl_error = packetHandler_2.write1ByteTxRx(portHandler, DXL6_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result_2 != COMM_SUCCESS:
    print("%s" % packetHandler_2.getTxRxResult(dxl_comm_result_2))
elif dxl_error != 0:
    print("%s" % packetHandler_2.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL6_ID)

'''
# Enable Dynamixel#2 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL2_ID)
'''
# Add parameter storage for Dynamixel#1 present position value
dxl_addparam_result = groupSyncRead.addParam(DXL1_ID)
if dxl_addparam_result != True:
    print("[ID:%03d] groupSyncRead addparam failed" % DXL1_ID)
    quit()

# Add parameter storage for Dynamixel#2 present position value
dxl_addparam_result = groupSyncRead.addParam(DXL2_ID)
if dxl_addparam_result != True:
    print("[ID:%03d] groupSyncRead addparam failed" % DXL2_ID)
    quit()
    
# Add parameter storage for Dynamixel#3 present position value
dxl_addparam_result = groupSyncRead.addParam(DXL3_ID)
if dxl_addparam_result != True:
    print("[ID:%03d] groupSyncRead addparam failed" % DXL3_ID)
    quit()

# Add parameter storage for Dynamixel#4 present position value
dxl_addparam_result_2 = groupSyncRead_2.addParam(DXL4_ID)
if dxl_addparam_result_2 != True:
    print("[ID:%03d] groupSyncRead addparam failed" % DXL4_ID)
    quit()

# Add parameter storage for Dynamixel#5 present position value
dxl_addparam_result_2 = groupSyncRead_2.addParam(DXL5_ID)
if dxl_addparam_result_2 != True:
    print("[ID:%03d] groupSyncRead addparam failed" % DXL5_ID)
    quit()
# Add parameter storage for Dynamixel#5 present position value    
dxl_addparam_result_2 = groupSyncRead_2.addParam(DXL6_ID)
if dxl_addparam_result_2 != True:
    print("[ID:%03d] groupSyncRead addparam failed" % DXL6_ID)
    quit()

amplitude = 50
frequency = 1

def Sine_gen(time, amplitude, frequency):
        Value = amplitude*math.sin(time*math.pi*frequency)
        return Value


    


    

while True:
    
    Value =Sine_gen(time.time(),amplitude,frequency)
    
    if Value >0:
        # Allocate goal position value into byte array
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[index])), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[index])), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[index])), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[index]))]
        param_goal_position_2 = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_2[index])), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_2[index])), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_2[index])), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_2[index]))]
        param_goal_position_3 = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_3[index])), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_3[index])), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_3[index])), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_3[index]))]
#         param_goal_position_4 = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_4[index])), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_4[index])), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_4[index])), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_4[index]))]
#         param_goal_position_5 = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_5[index])), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_5[index])), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_5[index])), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_5[index]))]
#         param_goal_position_6 = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_6[index])), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_6[index])), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_6[index])), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_6[index]))]
            
        # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_position)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL1_ID)
            quit()

        # Add Dynamixel#2 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_position_2)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL2_ID)
            quit()
    
        # Add Dynamixel#3 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWrite.addParam(DXL3_ID, param_goal_position_3)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL3_ID)
            quit()
            
        # Syncwrite goal position
        dxl_comm_result = groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                
        # Clear syncwrite parameter storage
        groupSyncWrite.clearParam()
        
        while 1:
            # Syncread present position
            dxl_comm_result = groupSyncRead.txRxPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

            # Check if groupsyncread data of Dynamixel#1 is available
            dxl_getdata_result = groupSyncRead.isAvailable(DXL1_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupSyncRead getdata failed" % DXL1_ID)
                quit()

            # Check if groupsyncread data of Dynamixel#2 is available
            dxl_getdata_result = groupSyncRead.isAvailable(DXL2_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupSyncRead getdata failed" % DXL2_ID)
                quit()
        
            # Check if groupsyncread data of Dynamixel#3 is available
            dxl_getdata_result = groupSyncRead.isAvailable(DXL3_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupSyncRead getdata failed" % DXL3_ID)
                quit()
            
#             # Check if groupsyncread data of Dynamixel#4 is available
#             dxl_getdata_result = groupSyncRead.isAvailable(DXL4_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
#             if dxl_getdata_result != True:
#                 print("[ID:%03d] groupSyncRead getdata failed" % DXL4_ID)
#                 quit()
#         
#             # Check if groupsyncread data of Dynamixel#5 is available
#             dxl_getdata_result = groupSyncRead.isAvailable(DXL5_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
#             if dxl_getdata_result != True:
#                 print("[ID:%03d] groupSyncRead getdata failed" % DXL5_ID)
#                 quit()
#             
#             # Check if groupsyncread data of Dynamixel#5 is available
#             dxl_getdata_result = groupSyncRead.isAvailable(DXL6_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
#             if dxl_getdata_result != True:
#                 print("[ID:%03d] groupSyncRead getdata failed" % DXL6_ID)
#                 quit()

            # Get Dynamixel#1 present position value
            dxl1_present_position = groupSyncRead.getData(DXL1_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

            # Get Dynamixel#2 present position value
            dxl2_present_position = groupSyncRead.getData(DXL2_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        
            # Get Dynamixel#3 present position value
            dxl3_present_position = groupSyncRead.getData(DXL3_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        
#             # Get Dynamixel#4 present position value
#             dxl4_present_position = groupSyncRead.getData(DXL4_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
#         
#             # Get Dynamixel#5 present position value
#             dxl5_present_position = groupSyncRead.getData(DXL5_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
#         
#             # Get Dynamixel#6 present position value
#             dxl6_present_position = groupSyncRead.getData(DXL6_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

            #print("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL1_ID, dxl_goal_position[index], dxl1_present_position, DXL2_ID, dxl_goal_position_2[index], dxl2_present_position))

            if not ((abs(dxl_goal_position[index] - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD) and
                    (abs(dxl_goal_position_2[index] - dxl2_present_position) > DXL_MOVING_STATUS_THRESHOLD) and
                    (abs(dxl_goal_position_3[index] - dxl3_present_position) > DXL_MOVING_STATUS_THRESHOLD)) :
                    break
            time.sleep(1)
            # Change goal position
        if index == 0:
            index = 1
        else:
            index = 0
            
    else:
        # Allocate goal position value into byte array
#         param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[index_2])), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[index_2])), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[index_2])), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[index_2]))]
#         param_goal_position_2 = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_2[index_2])), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_2[index_2])), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_2[index_2])), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_2[index_2]))]
#         param_goal_position_3 = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_3[index_2])), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_3[index_2])), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_3[index_2])), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_3[index_2]))]
        param_goal_position_4 = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_4[index])), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_4[index])), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_4[index])), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_4[index]))]
        param_goal_position_5 = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_5[index])), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_5[index])), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_5[index])), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_5[index]))]
        param_goal_position_6 = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_6[index])), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_6[index])), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_6[index])), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_6[index]))]
        # Add Dynamixel#4 goal position value to the Syncwrite parameter storage
        dxl_addparam_result_2 = groupSyncWrite_2.addParam(DXL4_ID, param_goal_position_4)
        if dxl_addparam_result_2 != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL4_ID)
            quit()
    
    
        
        # Add Dynamixel#5 goal position value to the Syncwrite parameter storage
        dxl_addparam_result_2 = groupSyncWrite_2.addParam(DXL5_ID, param_goal_position_5)
        if dxl_addparam_result_2 != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL5_ID)
            quit()
    
        # Add Dynamixel#6 goal position value to the Syncwrite parameter storage
        dxl_addparam_result_2 = groupSyncWrite_2.addParam(DXL6_ID, param_goal_position_6)
        if dxl_addparam_result_2 != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL6_ID)
            quit()
                
        # Syncwrite goal position
        dxl_comm_result_2 = groupSyncWrite_2.txPacket()
        if dxl_comm_result_2 != COMM_SUCCESS:
            print("%s" % packetHandler_2.getTxRxResult(dxl_comm_result_2))
            # Clear syncwrite parameter storage
        groupSyncWrite_2.clearParam()
        
#         # Change goal position
#         if index == 0:
#             index = 1
#         else:
#             index = 0
        
        
        while 1:
            # Syncread present position
            dxl_comm_result_2 = groupSyncRead_2.txRxPacket()
            if dxl_comm_result_2 != COMM_SUCCESS:
                print("%s" % packetHandler_2.getTxRxResult(dxl_comm_result_2))

#             # Check if groupsyncread data of Dynamixel#1 is available
#             dxl_getdata_result = groupSyncRead.isAvailable(DXL1_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
#             if dxl_getdata_result != True:
#                 print("[ID:%03d] groupSyncRead getdata failed" % DXL1_ID)
#                 quit()
# 
#             # Check if groupsyncread data of Dynamixel#2 is available
#             dxl_getdata_result = groupSyncRead.isAvailable(DXL2_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
#             if dxl_getdata_result != True:
#                 print("[ID:%03d] groupSyncRead getdata failed" % DXL2_ID)
#                 quit()
#         
#             # Check if groupsyncread data of Dynamixel#3 is available
#             dxl_getdata_result = groupSyncRead.isAvailable(DXL3_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
#             if dxl_getdata_result != True:
#                 print("[ID:%03d] groupSyncRead getdata failed" % DXL3_ID)
#                 quit()
            
            # Check if groupsyncread data of Dynamixel#4 is available
            dxl_getdata_result_2 = groupSyncRead_2.isAvailable(DXL4_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            if dxl_getdata_result_2 != True:
                print("[ID:%03d] groupSyncRead getdata failed" % DXL4_ID)
                quit()
        
            # Check if groupsyncread data of Dynamixel#5 is available
            dxl_getdata_result_2 = groupSyncRead_2.isAvailable(DXL5_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            if dxl_getdata_result_2 != True:
                print("[ID:%03d] groupSyncRead getdata failed" % DXL5_ID)
                quit()
            
            # Check if groupsyncread data of Dynamixel#5 is available
            dxl_getdata_result_2 = groupSyncRead_2.isAvailable(DXL6_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            if dxl_getdata_result_2 != True:
                print("[ID:%03d] groupSyncRead getdata failed" % DXL6_ID)
                quit()

#             # Get Dynamixel#1 present position value
#             dxl1_present_position = groupSyncRead.getData(DXL1_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
# 
#             # Get Dynamixel#2 present position value
#             dxl2_present_position = groupSyncRead.getData(DXL2_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
#         
#             # Get Dynamixel#3 present position value
#             dxl3_present_position = groupSyncRead.getData(DXL3_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        
            # Get Dynamixel#4 present position value
            dxl4_present_position = groupSyncRead_2.getData(DXL4_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        
            # Get Dynamixel#5 present position value
            dxl5_present_position = groupSyncRead_2.getData(DXL5_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        
            # Get Dynamixel#6 present position value
            dxl6_present_position = groupSyncRead_2.getData(DXL6_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

            #print("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL1_ID, dxl_goal_position[index], dxl1_present_position, DXL2_ID, dxl_goal_position_2[index], dxl2_present_position))

            if not ((abs(dxl_goal_position_4[index] - dxl4_present_position) > DXL_MOVING_STATUS_THRESHOLD) and
                    (abs(dxl_goal_position_5[index] - dxl5_present_position) > DXL_MOVING_STATUS_THRESHOLD) and
                    (abs(dxl_goal_position_6[index] - dxl6_present_position) > DXL_MOVING_STATUS_THRESHOLD)) :
                break
            time.sleep(1)
            # Change goal position
            if index == 0:
                index = 1
            else:
                index = 0
            
#  # Change goal position
#     if index == 0:
#         index = 1
#     else:
#         index = 0           
        
     
      

   

# Clear syncread parameter storage
groupSyncRead_2.clearParam()

# Disable Dynamixel#1 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Disable Dynamixel#2 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Disable Dynamixel#3 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
    
# Disable Dynamixel#4 Torque
dxl_comm_result_2, dxl_error = packetHandler_2.write1ByteTxRx(portHandler, DXL4_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result_2 != COMM_SUCCESS:
    print("%s" % packetHandler_2.getTxRxResult(dxl_comm_result_2))
elif dxl_error != 0:
    print("%s" % packetHandler_2.getRxPacketError(dxl_error))

# Disable Dynamixel#5 Torque
dxl_comm_result_2, dxl_error = packetHandler_2.write1ByteTxRx(portHandler, DXL5_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result_2 != COMM_SUCCESS:
    print("%s" % packetHandler_2.getTxRxResult(dxl_comm_result_2))
elif dxl_error != 0:
    print("%s" % packetHandler_2.getRxPacketError(dxl_error))

# Disable Dynamixel#6 Torque
dxl_comm_result_2, dxl_error = packetHandler_2.write1ByteTxRx(portHandler, DXL6_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result_2 != COMM_SUCCESS:
    print("%s" % packetHandler_2.getTxRxResult(dxl_comm_result_2))
elif dxl_error != 0:
    print("%s" % packetHandler_2.getRxPacketError(dxl_error))
'''
# Disable Dynamixel#2 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))'''

# Close port
portHandler.closePort()



