from dynamixel_sdk import *                    # Uses Dynamixel SDK library
import time
import math

MY_DXL = 'P_SERIES'     # PH54, PH42, PM54

# Control table address

if MY_DXL == 'P_SERIES' or MY_DXL == 'PRO_A_SERIES':
    ADDR_TORQUE_ENABLE          = 512        # Control table address is different in DYNAMIXEL model
    ADDR_GOAL_POSITION          = 564
    LEN_GOAL_POSITION           = 4          # Data Byte Length
    ADDR_PRESENT_POSITION       = 580
    LEN_PRESENT_POSITION        = 4          # Data Byte Length
    DXL_MINIMUM_POSITION_VALUE  = -501433     # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 501433      # Refer to the Maximum Position Limit of product eManual
    BAUDRATE                    = 1000000

# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION            = 2.0

# Make sure that each DYNAMIXEL ID should have unique ID.
DXL_ID = [0, 1, 2, 3, 4, 5]
pos = [0, 0, 0, 0, 0, 0]
ldxl = len(DXL_ID) 
DXL1_ID                     = 1                 # Dynamixel#1 ID : 1
DXL2_ID                     = 0                 # Dynamixel#1 ID : 2

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = '/dev/ttyUSB0'

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupSyncWrite instance
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

# Initialize GroupSyncRead instace for Present Position
groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)



def check(pos):
    cnt = 0
    for x in range(ldxl):
        dxl_present_position = groupSyncRead.getData(DXL_ID[x], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        if pos[x] < 0:
            diff = pos[x]*2789 - dxl_present_position + 2**32
        else:
            diff = pos[x]*2789 - dxl_present_position
        print(x, diff)
        if not (abs(diff) > DXL_MOVING_STATUS_THRESHOLD):
            cnt += 1
     
    if cnt == ldxl:
        return True
    else:
        return False

def move(pos):
    if len(pos) < ldxl:
        print("not enough")
    else:
        print("fine")
        
        # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
        for x in range (ldxl):
            param_goal_position = [DXL_LOBYTE(DXL_LOWORD(pos[x]*2789)), DXL_HIBYTE(DXL_LOWORD(pos[x]*2789)), DXL_LOBYTE(DXL_HIWORD(pos[x]*2789)), DXL_HIBYTE(DXL_HIWORD(pos[x]*2789))]
            dxl_addparam_result = groupSyncWrite.addParam(DXL_ID[x], param_goal_position)

            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % DXL_ID[x])
                quit()

        # Add Dynamixel#2 goal position value to the Syncwrite parameter storage
        '''dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_position)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL2_ID)
            quit()
    '''
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
            for x in range(ldxl):
                dxl_getdata_result = groupSyncRead.isAvailable(DXL_ID[x], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
                if dxl_getdata_result != True:
                    print("[ID:%03d] groupSyncRead getdata failed" % DXL_ID[x])
                    quit()

            # Get Dynamixel#1 present position value
            if check(pos):
                break
        

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

for x in range(ldxl):
# Enable Dynamixel#1 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[x], ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % DXL_ID[x])

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
for x in range (ldxl):
    dxl_addparam_result = groupSyncRead.addParam(DXL_ID[x])
    #print(dxl_addparam_result)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncRead addparam failed" % DXL_ID[x])
        quit()

# Add parameter storage for Dynamixel#2 present position value
'''dxl_addparam_result = groupSyncRead.addParam(DXL2_ID)
if dxl_addparam_result != True:
    print("[ID:%03d] groupSyncRead addparam failed" % DXL2_ID)
    quit()
'''    #0   1  2  3  4  5
pos = [[0, -90, 179, 0, 0, 179],
       [0, -90, 0, 0, 0, 179],
       [0, -90, 120, 0, 0, 179],
       [0, -90, 120, 0, 0, 179]]

num = 0
while 1:
    #pos[1] = int(200000.0*math.sin(2*math.pi*.25*time.time()))
    # Allocate goal position value into byte array
    print(pos[num%2])
    move(pos[num%2])
    num += 1



    # Change goal position


# Clear syncread parameter storage
groupSyncRead.clearParam()

for x in range(ldxl):
# Disable Dynamixel#1 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
'''
# Disable Dynamixel#2 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))'''

# Close port
portHandler.closePort()

