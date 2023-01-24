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

def poly(in_angle, fin_angle, t):
    pos = [1, 1, 1, 1, 1, 1]
    vel = [0, 1, 2, 3, 4, 5]
    accel = [0, 0, 2, 6, 12, 20]
    a_coeff = []
    first = [1,  t, t**2, t**3, t**4, t**5, fin_angle]
    second = [0, 1, t * 2, 3 * t**2, 4 * t**3, 5 * t**4, 0]
    third = [0, 0, 2, 6 * t, 12 * t**2, 20 * t**3, 0]
    fourth = [1, 0, 0, 0, 0, 0, in_angle]
    fifth = [0, 1, 0, 0, 0, 0, 0]
    sixth = [0, 0, 1, 0, 0, 0, 0]
    full = [fourth, fifth, sixth, first, second, third]

    for i in range(6):
        div = full[i][i]
        for x in range (len(full[i])):
            full[i][x] /= div
        for x in range(i + 1, len(full)):
            if full[x][i] != 0:
                mult = full[x][i]
                for y in range (i, len(full[x])):
                    full[x][y] -= (full[i][y]*mult)
                    
    for j in range(2):
        mult = full[5][5]/full[4 - j][5]
        for i in range(2):
            full[4 - j][6 - i] -= full[5][6 - i]/mult

    mult = full[4][4]/full[3][4]
    for i in range(3):
        full[3][6 - i] -= full[4][6 - i]/mult

    for x in range(len(full)):
        a_coeff.append(full[x][6])

    return a_coeff

def move_many(arr, t):
    a= []
    ts = 0
#     for x in arr:
#         if len(x) != 3:
#             print("WRONG ARRAY")
#             return -1
    for x in arr:
        a.append(poly(x[0], x[1], t))
    t0 = time.time()
    k = 0
    while k == 0:
        pos = []
        for x in range(len(arr)):
            t1 = time.time()
            if (t1 - t0 > t):
                k = 1
            t_diff  = t1 - t0
            pos.append(int((a[x][0] + a[x][1] * t_diff + a[x][2] * t_diff ** 2 + a[x][3] * t_diff ** 3 + a[x][4] * t_diff ** 4 + a[x][5] * t_diff ** 5)*2789))
            #poss = int(pos * 2789)
        if k == 1:
            break
        for x in range(len(arr)):
            param_goal_position = [DXL_LOBYTE(DXL_LOWORD(pos[x])), DXL_HIBYTE(DXL_LOWORD(pos[x])), DXL_LOBYTE(DXL_HIWORD(pos[x])), DXL_HIBYTE(DXL_HIWORD(pos[x]))]
            dxl_addparam_result = groupSyncWrite.addParam(DXL_ID[arr[x][-1]], param_goal_position)
            dxl_comm_result = groupSyncWrite.txPacket()

        groupSyncWrite.clearParam()
        try:
            time.sleep(0.008 - time.time() + t1)
        except:
            print(-1)
        
def move_poly(start, end, t, m_id):
    a = poly(start, end, t)
    t0 = time.time()
    p = []
    while 1:
        t1 = time.time()
        if (t1 - t0 > t):
            break
        t_diff  = t1 - t0
        pos = a[0] + a[1] * t_diff + a[2] * t_diff ** 2 + a[3] * t_diff ** 3 + a[4] * t_diff ** 4 + a[5] * t_diff ** 5
        poss = int(pos * 2789)
        p.append(int(pos))
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(poss)), DXL_HIBYTE(DXL_LOWORD(poss)), DXL_LOBYTE(DXL_HIWORD(poss)), DXL_HIBYTE(DXL_HIWORD(poss))]
        dxl_addparam_result = groupSyncWrite.addParam(DXL_ID[m_id], param_goal_position)
        
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL_ID[x])
            quit()
        dxl_comm_result = groupSyncWrite.txPacket()
        #if dxl_comm_result != COMM_SUCCESS:
            #print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        groupSyncWrite.clearParam()
        try:
            time.sleep(0.002 - time.time() + t1)
        except:
            pass
        #print(t_diff)
    print(p, len(p))
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
pos = [[-60, -120, 179, -179, -179, 179],
       [-179, 0, 30, 60, -30, -60],
       [0, -90, 120, 0, 0, 179],
       [0, -90, 120, 0, 0, 179]]

R_trans1 = [[-120, 0, 1],
          [179, -30, 2],
          [179, -60, 5]]

R_trans2 = [[0, -120, 1],
          [-30, 179, 2],
          [-60, 179, 5]]



L_trans1 =[[-60, -179, 0],
         [-179, 60, 3],
         [-179, 60, 4]]

L_trans2 = [[-179, -60, 0],
          [60, -179, 3],
          [60, -179, 4]]



wheel1 = [[-179, -30, 0],
          [0, -150, 1]]

wheel2 = [[-45, -179, 0],
          [-135, 0, 1]]

leg1 = [[30, 179, 2],
        [-30, -179, 4]]

leg2 = [[179, 30, 2],
        [-179, -30, 4]]

shldr1 = [[60, -179, 3],
          [-60, 179, 5]]

shldr2 = [[-179, 60, 3],
          [179, -60, 5]]
fin = [[-30, -60, 0],
       [-150, -120, 1]]


step = [[-60, -120, 0],
        [-120, -60, 1]]

R_trans1_s = [[-120, 0, 1],
          [179, 0, 2],
          [179, -60, 5]]

R_trans2_s = [[0, -120, 1],
          [0, 179, 2],
          [-60, 179, 5]]

L_trans1_s =[[-60, -179, 0],
         [-179, 85, 3],
         [-179, 20, 4]]

L_trans2_s= [[-179, -60, 0],
          [85, -179, 3],
          [20, -179, 4]]


###new transition: legged to wheeled mode
upper_joint_up = [[179, -60, 5],
           [-179, 85, 3]]

lower_joint_up = [[179, 0, 2],
                  [-179, 20, 4]]

legs_to_back = [[-120, 0, 1],
                [-60, -179, 0]]

###transition from wheeled to legged mode:
legs_to_front = [[0, -120, 1],
                 [-179, -60, 0]]

lower_joint_down = [[0, 179, 2],
                    [20, -179, 4]]

upper_joint_down = [[-60, 179, 5],
                         [85, -179, 3]]



time.sleep(10)
#move_many(step, 2)
##move_many(L_trans1_s, 3)
##time.sleep(10)
##move_many(L_trans2_s, 3)

##move_many(R_trans1_s, 3)
##time.sleep(10)
##move_many(R_trans2_s, 3)

move_many(upper_joint_up, 3)
time.sleep(2)
move_many(lower_joint_up, 3)
time.sleep(2)
move_many(legs_to_back, 3)

time.sleep(10)
move_many(legs_to_front, 3)
time.sleep(2)
move_many(lower_joint_down, 3)
time.sleep(2)
move_many(upper_joint_down, 3)
time.sleep(2)


##move_poly(179, 0, 3, 2)
##time.sleep(1)
##move_poly(0, 179, 3, 2)
##move_poly(-179, 20, 3, 4) #with shifted ring
##time.sleep(1)
##move_poly(20, -179, 3, 4)

#move_many(L_trans1, 2)
#time.sleep(5)
#move_many(wheel1, 2)
#move_many(leg1, 2)
#move_many(shldr1, 1)
#move_many(fin, 0.5)
#move_poly(-135, -120, 1, 1)
#move_poly(-45, -60, 1, 0)
#move_many(leg1, 2)
#time.sleep(5)
#move_many(wheel2, 2)
#move(pos[1])
# Close port
portHandler.closePort()
