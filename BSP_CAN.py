import socket, struct, sys, json, time, os.path, threading, math
import serial, select, copy, pdb, pyvesc as esc
import paho.mqtt.client as mqtt
import BSP_ERROR, BSP_PID as PID
import BSP_ROBOT_CONFIG as ROB

rob = ROB.robot()

PRINT_MOTOR_INFO = False
PRINT_ROLLING = False
PRINT_RANGE = [5]

PRINT_Motor_Angle = True
PRINT_Motor_Speed = False
PRINT_Motor_Torque = True
PRINT_Upper_Output = False
PRINT_Lower_Output = False
PRINT_Control_Signal = True
PRINT_Angle_Massage = False
PRINT_Speed_Massage = False
PRINT_Torque_Massage = False

ENABLE_Control_From_Shooter =   False
ENABLE_Control_From_Decision =  False
ENABLE_Control_From_Remote =    True

CHASSIS_SPEED_INDEX = 100
MOTOR_ID_HEX = [0x201, 0x202, 0x203, 0x204, 0x205, 0x206, 0x207]

Remote_Pitch = 0
Remote_Yaw = 0

FEEDER_POS_TURN = 2300
FEEDER_REV_TURN = -600

PREVIOUS_SHOOT_TIME_COUNT = 0

init = []
for i in range(rob.mono):
    init.append(True)

SHOTTER_MOTOR_REVERSE = False

version = "01A00B " + time.ctime(os.path.getctime(os.sys.argv[0]))
PID_Num = len(rob.PID_Items)

YAW_ANGLE = 0.0
PITCH_ANGLE = 0.0
YAW_OMEGA = 0.0
PITCH_OMEGA = 0.0


SERIAL_COMM = []

PID_SETTINGS_SET = []
PID_SETTINGS_SET.append({"P":0.0, "I":0.0, "D":0.0})             #Chassis_Upper
PID_SETTINGS_SET.append({"P":0.0 ,"I":0.0, "D":0.0})            #Chassis_Lower

PID_SETTINGS_SET.append({"P":0.0 ,"I":0.0, "D":0.0})      #Yaw_Upper
PID_SETTINGS_SET.append({"P":0.0 ,"I":0.0, "D":0.0})            #Yaw_Lower

PID_SETTINGS_SET.append({"P":0.0 ,"I":0.0 ,"D":0.0})       #Pitch_Upper
PID_SETTINGS_SET.append({"P":0.0 ,"I":0.0, "D":0.0})            #Pitch_Lower

PID_SETTINGS_SET.append({"P":0.0 ,"I":0.0, "D":0.0})             #Feeding_Upper
PID_SETTINGS_SET.append({"P":0.0 ,"I":0.0, "D":0.0})            #Feeding_Lower

PID_SETTINGS_REAL = copy.deepcopy(PID_SETTINGS_SET)

MOTOR_UPPER_SETTINS = []
MOTOR_LOWER_SETTINS = []

for i in range(4):
    MOTOR_UPPER_SETTINS.append(PID_SETTINGS_REAL[0])
    MOTOR_LOWER_SETTINS.append(PID_SETTINGS_REAL[1])
MOTOR_UPPER_SETTINS.append(PID_SETTINGS_REAL[2])
MOTOR_LOWER_SETTINS.append(PID_SETTINGS_REAL[3])
MOTOR_UPPER_SETTINS.append(PID_SETTINGS_REAL[4])
MOTOR_LOWER_SETTINS.append(PID_SETTINGS_REAL[5])
MOTOR_UPPER_SETTINS.append(PID_SETTINGS_REAL[6])
MOTOR_LOWER_SETTINS.append(PID_SETTINGS_REAL[7])


MOTOR_UPPER = []
MOTOR_UPPER_SetPoints = [0, 0, 0, 0, 174, 174, 0]
MOTOR_UPPER_RANGES = [0,0,0,0,360,360,0]

for i in range(rob.mono):
    MOTOR_UPPER.append(PID.PID(MOTOR_UPPER_SETTINS[i]["P"], MOTOR_UPPER_SETTINS[i]["I"], MOTOR_UPPER_SETTINS[i]["D"], MOTOR_UPPER_RANGES[i]))
    MOTOR_UPPER[i].SetPoint=MOTOR_UPPER_SetPoints[i]
    MOTOR_UPPER[i].setSampleTime(0.01)

MOTOR_UPPER[4].setDegreeFixer(True)
MOTOR_UPPER[5].setDegreeFixer(True)



MOTOR_LOWER = []
MOTOR_LOWER_SetPoints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100]
MOTOR_OUT_LIMIT = [32768, 32768, 32768, 32768, 5000, 5000, 32768]
MOTOR_LOWER_RANGES = [0,0,0,0,200,200,0]

for i in MOTOR_OUT_LIMIT:
    assert i <= 2**15 # THE MAX ABC_LOWER IS 2**15

for i in range(rob.mono):
    MOTOR_LOWER.append(PID.PID(MOTOR_LOWER_SETTINS[i]["P"], MOTOR_LOWER_SETTINS[i]["I"], MOTOR_LOWER_SETTINS[i]["D"], MOTOR_LOWER_RANGES[i]))
    MOTOR_LOWER[i].SetPoint=MOTOR_LOWER_SetPoints[i]
    MOTOR_LOWER[i].setSampleTime(0.01)


SKIP_UPPER = [False,False,False,False,False,False,True]

SKIP_LOWER = [True, True, True, True, False, False, False]


print(BSP_ERROR.access("BSP CAN START RUNNING, Version:" + version))
fmt = "<IB3x8s" #Regex for CAN Protocol

print(BSP_ERROR.info("Socket CAN Interface Start Binding."))
sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) #Socket CAN
interface = "can0"

def update_PID():
    for i in range(rob.mono):
        MOTOR_UPPER[i].clear()
        MOTOR_UPPER[i].setKp(MOTOR_UPPER_SETTINS[i]["P"])
        MOTOR_UPPER[i].setKi(MOTOR_UPPER_SETTINS[i]["I"])
        MOTOR_UPPER[i].setKd(MOTOR_UPPER_SETTINS[i]["D"])
        MOTOR_LOWER[i].clear()
        MOTOR_LOWER[i].setKp(MOTOR_LOWER_SETTINS[i]["P"])
        MOTOR_LOWER[i].setKi(MOTOR_LOWER_SETTINS[i]["I"])
        MOTOR_LOWER[i].setKd(MOTOR_LOWER_SETTINS[i]["D"])


def empty_socket(sock):
    """remove the data present on the socket"""
    input = [sock]
    while 1:
        inputready, o, e = select.select(input,[],[], 0.0)
        if len(inputready)==0: break
        for s in inputready: s.recv(1)

try:
    sock.bind((interface,))
    empty_socket(sock)
except OSError:
    print(BSP_ERROR.fail("Could not bind to interface '%s'\n" % interface))
    exit()

print(BSP_ERROR.notice("Socker CAN Interface Binding Success."))

def FeederReverseTurn():
    #TODO: This is not OOP at all !!!
    MOTOR_UPPER[6].SetPoint = MOTOR_UPPER[6].SetPoint + FEEDER_REV_TURN

def on_connect(client, userdata, flags, rc):
    print(BSP_ERROR.notice("MQTT Interface Bind Success."))
    client.subscribe("/CANBUS/#")
    client.subscribe("/REMOTE/#")
    client.subscribe("/SHOOTER/PUB/#")
    client.subscribe("/PID_REMOTE/#")
    client.subscribe("/CONFIG/#")
    client.subscribe("/IMU/AHRS")

    print(BSP_ERROR.notice("MQTT Subscribe Success, Topic: /CANBUS/#, Start Receiving CAN Messages."))
    Can_thread = CanThread()
    Can_thread.start()
    # t = threading.Thread(target = CAN_RCV_LOOP)
    # t.start()


MsgPayload = {"/REMOTE/": {}, "/CONFIG/": {}, "/SHOOTER/PUB/": {}, "/PID_REMOTE/": {}, "/REMOTE/SWITCH": {}, "/IMU/AHRS": {}, "/REMOTE/EXP": {}}
OnIMU = False
IMUpayload = {}

def on_message(client, userdata, msg):
    global OnIMU
    global IMUpayload
    global ENABLE_Control_From_Remote
    global YAW_ANGLE
    global PITCH_ANGLE
    global YAW_OMEGA
    global PITCH_OMEGA
    # print(BSP_ERROR.info("Topic: "+ msg.topic + " Payload: " + msg.payload.decode("utf-8")))
    payload = json.loads(msg.payload.decode("utf-8"))
    # print(MsgTopic)

    

    if msg.topic == "/REMOTE/" and ENABLE_Control_From_Remote:
        Robot_X = payload["XSpeed"]
        Robot_Y = payload["YSpeed"]
        Robot_Phi = payload["PhiSpeed"]
        Remote_Yaw = payload["Yaw"]
        Remote_Pitch = payload["Pitch"]

        MOTOR_UPPER[4].SetPoint = payload["YawAngle"]
        MOTOR_UPPER[5].SetPoint = payload["PitchAngle"]
        # MOTOR_UPPER[6].SetPoint = MOTOR_UPPER[6].SetPoint + payload["Pos"]
        MOTOR_Remote = [-Robot_X+Robot_Y+Robot_Phi, Robot_X+Robot_Y+Robot_Phi, Robot_X-Robot_Y+Robot_Phi, -Robot_X-Robot_Y+Robot_Phi]
        for i in range(4):
            MOTOR_UPPER[i].SetPoint = MOTOR_Remote[i]*CHASSIS_SPEED_INDEX

    elif msg.topic == "/PID_REMOTE/":
        Ps = payload.get("Ps")
        Is = payload.get("Is")
        Ds = payload.get("Ds")
        for i in range(PID_Num):
            PID_SETTINGS_REAL[i]["P"] = Ps[i]
            PID_SETTINGS_REAL[i]["I"] = Is[i]
            PID_SETTINGS_REAL[i]["D"] = Ds[i]
        print(str(PID_SETTINGS_REAL[0]["P"]))
        update_PID()
        publish_real_pid()

    elif msg.topic == "/CONFIG/":
        Config_Type = payload["Type"]
        Config_Set = payload["Set"]
        for i in range(rob.mono):
            if Config_Type == "Upper":
                SKIP_UPPER[i] = False
                MOTOR_UPPER[i].SetPoint = Config_Set[i]
            elif Config_Type == "Lower":
                SKIP_UPPER[i] = True
                MOTOR_LOWER[i].SetPoint = Config_Set[i]

    elif msg.topic == "/SHOOTER/PUB/":
        global PREVIOUS_SHOOT_TIME_COUNT
        MOTOR_UPPER[4].SetPoint = MOTOR_UPPER[4].SetPoint + payload["YawPhi"]
        MOTOR_UPPER[5].SetPoint = MOTOR_UPPER[5].SetPoint + payload["PitchPhi"]

        if payload["ShootStatus"] == "Fire" and time.time() - PREVIOUS_SHOOT_TIME_COUNT > 0.1:
            PREVIOUS_SHOOT_TIME_COUNT = time.time()
            MOTOR_UPPER[6].SetPoint = MOTOR_UPPER[6].SetPoint + FEEDER_POS_TURN
            s = threading.Timer(0.1, FeederReverseTurn)
            s.start()


    elif msg.topic == "/REMOTE/SWITCH":
        ENABLE_Control_From_Decision = payload["CTRL_Decision"]
        ENABLE_Control_From_Remote = payload["CTRL_Remote"]
        ENABLE_Control_From_Shooter = payload["CTRL_Shooter"]


    elif msg.topic == "/REMOTE/EXP":
        MOTOR_UPPER[4].SetPoint = payload["YawAngle"]
        MOTOR_UPPER[5].SetPoint = payload["PitchAngle"]
        MOTOR_UPPER[6].SetPoint = MOTOR_UPPER[6].SetPoint + payload["Pos"]


    elif msg.topic == "/IMU/AHRS":
        YAW_ANGLE = payload["Yaw"]
        PITCH_ANGLE = payload["Pitch"]
        YAW_OMEGA = payload["GyroYaw"]
        PITCH_OMEGA = payload["GyroPitch"]
        # OnIMU = True
        # print("f")
        # IMUpayload = payload
    

    

def massageProcess():
    global MsgPayload
    global ENABLE_Control_From_Remote
    # print("Msg Processing")
    # print(MsgTopic)
    # if MsgPayload["/REMOTE/"] != {} and ENABLE_Control_From_Remote:
    #     payload = MsgPayload["/REMOTE/"]
    #     Robot_X = payload["XSpeed"]
    #     Robot_Y = payload["YSpeed"]
    #     Robot_Phi = payload["PhiSpeed"]
    #     Remote_Yaw = payload["Yaw"]
    #     Remote_Pitch = payload["Pitch"]

    #     MOTOR_UPPER[4].SetPoint = payload["YawAngle"]
    #     MOTOR_UPPER[5].SetPoint = payload["PitchAngle"]
    #     # MOTOR_UPPER[6].SetPoint = MOTOR_UPPER[6].SetPoint + payload["Pos"]
    #     MOTOR_Remote = [-Robot_X+Robot_Y+Robot_Phi, Robot_X+Robot_Y+Robot_Phi, Robot_X-Robot_Y+Robot_Phi, -Robot_X-Robot_Y+Robot_Phi]
    #     for i in range(4):
    #         MOTOR_UPPER[i].SetPoint = MOTOR_Remote[i]*CHASSIS_SPEED_INDEX
    #     MsgPayload["/REMOTE/"] = {}

    # if MsgPayload["/PID_REMOTE/"] != {}:
    #     payload = MsgPayload["/PID_REMOTE/"]
    #     Ps = payload.get("Ps")
    #     Is = payload.get("Is")
    #     Ds = payload.get("Ds")
    #     for i in range(PID_Num):
    #         PID_SETTINGS_REAL[i]["P"] = Ps[i]
    #         PID_SETTINGS_REAL[i]["I"] = Is[i]
    #         PID_SETTINGS_REAL[i]["D"] = Ds[i]
    #     print(str(PID_SETTINGS_REAL[0]["P"]))
    #     update_PID()
    #     publish_real_pid()
    #     MsgPayload["/PID_REMOTE/"] = {}

    # if MsgPayload["/CONFIG/"] != {}:
    #     payload = MsgPayload["/CONFIG/"]
    #     Config_Type = payload["Type"]
    #     Config_Set = payload["Set"]
    #     for i in range(rob.mono):
    #         if Config_Type == "Upper":
    #             SKIP_UPPER[i] = False
    #             MOTOR_UPPER[i].SetPoint = Config_Set[i]
    #         elif Config_Type == "Lower":
    #             SKIP_UPPER[i] = True
    #             MOTOR_LOWER[i].SetPoint = Config_Set[i]
    #     MsgPayload["/CONFIG/"] = {}

    # if MsgPayload["/SHOOTER/PUB/"] != {}:
    #     payload = MsgPayload["/SHOOTER/PUB/"]
    #     global PREVIOUS_SHOOT_TIME_COUNT
    #     MOTOR_UPPER[4].SetPoint = MOTOR_UPPER[4].SetPoint + payload["YawPhi"]
    #     MOTOR_UPPER[5].SetPoint = MOTOR_UPPER[5].SetPoint + payload["PitchPhi"]

    #     if payload["ShootStatus"] == "Fire" and time.time() - PREVIOUS_SHOOT_TIME_COUNT > 0.1:
    #         PREVIOUS_SHOOT_TIME_COUNT = time.time()
    #         MOTOR_UPPER[6].SetPoint = MOTOR_UPPER[6].SetPoint + FEEDER_POS_TURN
    #         s = threading.Timer(0.1, FeederReverseTurn)
    #         s.start()
    #     MsgPayload["/SHOOTER/PUB/"] = {}


    # if MsgPayload["/REMOTE/SWITCH"] != {}:
    #     payload = MsgPayload["/REMOTE/SWITCH"]
    #     ENABLE_Control_From_Decision = payload["CTRL_Decision"]
    #     ENABLE_Control_From_Remote = payload["CTRL_Remote"]
    #     ENABLE_Control_From_Shooter = payload["CTRL_Shooter"]
    #     MsgPayload["/REMOTE/SWITCH"] = {}


    # if MsgPayload["/REMOTE/EXP"] != {}:
    #     payload = MsgPayload["/REMOTE/EXP"]
    #     MOTOR_UPPER[4].SetPoint = payload["YawAngle"]
    #     MOTOR_UPPER[5].SetPoint = payload["PitchAngle"]
    #     MOTOR_UPPER[6].SetPoint = MOTOR_UPPER[6].SetPoint + payload["Pos"]
    #     MsgPayload["/REMOTE/EXP"] = {}

    

def imuMassageProcess():
    global OnIMU
    global IMUpayload
    if OnIMU:#MsgPayload["/IMU/AHRS"] != {}:
        payload = IMUpayload# MsgPayload["/IMU/AHRS"]
        global YAW_ANGLE
        global PITCH_ANGLE
        global YAW_OMEGA
        global PITCH_OMEGA
        YAW_ANGLE = float(payload["Yaw"])
        PITCH_ANGLE = float(payload["Pitch"])
        YAW_OMEGA = float(payload["GyroYaw"])
        PITCH_OMEGA = float(payload["GyroPitch"])
        # print("imu updated")
        # MsgPayload["/IMU/AHRS"] = {}
        OnIMU = False
        print("u")
    


def compare_pid():
    for i in range(int(PID_Num)):
        if PID_SETTINGS_REAL[i]["P"] != PID_SETTINGS_SET[i]["P"] or PID_SETTINGS_REAL[i]["I"] != PID_SETTINGS_SET[i]["I"] or PID_SETTINGS_REAL[i]["D"] != PID_SETTINGS_SET[i]["D"]:
            return "False"
    return "True"

def publish_real_pid():
    #pdb.set_trace()
    Ps = []
    Is = []
    Ds = []
    for i in range(int(PID_Num/2)):
        Ps.append(MOTOR_UPPER[3+i].getP())
        Is.append(MOTOR_UPPER[3+i].getI())
        Ds.append(MOTOR_UPPER[3+i].getD())

        Ps.append(MOTOR_LOWER[3+i].getP())
        Is.append(MOTOR_LOWER[3+i].getI())
        Ds.append(MOTOR_LOWER[3+i].getD())
    agree = compare_pid()
    pid_msg = {"Ps":Ps, "Is":Is, "Ds":Ds, "Agree": agree}
    client.publish("/PID_FEEDBACK/", json.dumps(pid_msg))

class MsgThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)

    def run(self):
        while 1:
            massageProcess()

class Msg_imu_Thread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)

    def run(self):
        while 1:
            imuMassageProcess()

class CanThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)

    def run(self):
        CAN_RCV_LOOP()


def get_sign(num):
    if num >= 0:
        return " "
    else:
        return "-"

def CAN_RCV_LOOP():
    MOTOR_Angle = []
    MOTOR_Phi = []
    MOTOR_Now = []
    motor_out = []
    MOTOR_Total = []
    MOTOR_Updated = [False,False,False,False,True,True,True]
    MOTOR_ID_DES = []
    MOTOR_ANGLE_MSG_OUT = []
    MOTOR_SPEED_MSG_OUT = []
    MOTOR_TORQUE_MSG_OUT = []
    MOTOR_TIMER = []
    MOTOR_OMEGA = []
    MOTOR_Torque = []
    UPPER_OUT = []
    LOWER_OUT = []

    for i in range(7):
        MOTOR_TIMER.append(time.time())
        MOTOR_Angle.append(0.0)
        MOTOR_Phi.append(0.0)
        MOTOR_Now.append(0.0)
        motor_out.append(0.0)
        MOTOR_Total.append(0.0)
        # MOTOR_Updated.append(False)
        MOTOR_ID_DES.append(i)
        MOTOR_ANGLE_MSG_OUT.append(0.0)
        MOTOR_SPEED_MSG_OUT.append(0.0)
        MOTOR_TORQUE_MSG_OUT.append(0.0)
        MOTOR_OMEGA.append(0.0)
        MOTOR_Torque.append(0.0)
        UPPER_OUT.append(0.0)
        LOWER_OUT.append(0.0)

    mqtt_count = 0
    phi_count = [0,0,0,0,0,0,0]
    printcount = 0

    while 1:
        # print("can loop")
        can_pkt = sock.recv(16)
        can_id, length, data = struct.unpack(fmt, can_pkt)
        can_id &= socket.CAN_EFF_MASK

        if length < 8:
            print("Broken CAN Package Detected, DROP;")
            continue

        data = data[:length]

        if len(data) < 6: #Maybe a currupt can package
            print(BSP_ERROR.fail("Currupt CAN Package Detected. Length: %d" % len(data)))
            continue

        if can_id in MOTOR_ID_HEX:

            torque = data[4]*256+data[5]
            if torque >= 2**15:
                torque = torque-2**16
            speed = data[2]*256+data[3]
            if speed >= 2**15:
                speed = speed-2**16
            TIME_NOW = time.time()



            for i in range(rob.mono):
                if can_id == MOTOR_ID_HEX[i] :
                    if TIME_NOW - MOTOR_TIMER[i] > 0.01:#phi_count[i] > 10: #reduce the speed of phi
                        MOTOR_Torque[i] = torque
                        MOTOR_Now[i] = (360.0)/(8191)*(data[0]*256+data[1])
                        if init[i]:
                            MOTOR_Angle[i] = MOTOR_Now[i]
                            init[i] = False
                        MOTOR_Phi[i] = MOTOR_Now[i] - MOTOR_Angle[i]
                        if MOTOR_Phi[i] > 180:
                            MOTOR_Phi[i] = MOTOR_Phi[i] - 360
                        elif MOTOR_Phi[i] < -180:
                            MOTOR_Phi[i] = MOTOR_Phi[i] + 360

                        MOTOR_OMEGA[i] = MOTOR_Phi[i]/(TIME_NOW - MOTOR_TIMER[i])

                        # if i in range(4,5):
                            # print(str(TIME_NOW - MOTOR_TIMER[i])+ " " +str(MOTOR_OMEGA[i])+" "+str(MOTOR_Phi[i]*10))


                        MOTOR_TIMER[i] = TIME_NOW
                        MOTOR_Angle[i] = MOTOR_Now[i]
                        MOTOR_Total[i] = MOTOR_Total[i] + MOTOR_Phi[i]

                        if i == 4:
                            MOTOR_OMEGA[i] = YAW_OMEGA
                            # MOTOR_Angle[i] = YAW_ANGLE
                        elif i == 5:
                            MOTOR_OMEGA[i] = PITCH_OMEGA
                            # MOTOR_Angle[i] = PITCH_ANGLE


                        if rob.UPPER_PID_TYPE[i] == "SPD":
                            MOTOR_UPPER[i].update(MOTOR_OMEGA[i])
                        elif rob.UPPER_PID_TYPE[i] == "ANG":
                            MOTOR_UPPER[i].update(MOTOR_Angle[i])
                        elif rob.UPPER_PID_TYPE[i] == "FEED":
                            MOTOR_UPPER[i].update(MOTOR_Total[i])

                        if not SKIP_UPPER[i]:
                            UPPER_OUT[i] = MOTOR_UPPER[i].output
                            if UPPER_OUT[i] > MOTOR_LOWER_RANGES[i]:
                                UPPER_OUT[i] = MOTOR_LOWER_RANGES[i]
                            elif UPPER_OUT[i] < -MOTOR_LOWER_RANGES[i]:
                                UPPER_OUT[i] = -MOTOR_LOWER_RANGES[i]
                            MOTOR_LOWER[i].SetPoint = UPPER_OUT[i]

                        if rob.LOWER_PID_TYPE[i] == "SPD":
                            MOTOR_LOWER[i].update(MOTOR_OMEGA[i])
                        elif rob.LOWER_PID_TYPE[i] == "TRQ":
                            MOTOR_LOWER[i].update(torque)
                            pass


                        phi_count[i] = 0
                    else:
                        phi_count[i] = phi_count[i] + 1

                    if SKIP_LOWER[i]:
                        UPPER_OUT[i] = MOTOR_UPPER[i].output
                        motor_out[i] = MOTOR_UPPER[i].output
                        if UPPER_OUT[i] > MOTOR_OUT_LIMIT[i]:
                            UPPER_OUT[i] = MOTOR_OUT_LIMIT[i]
                        elif UPPER_OUT[i] < -MOTOR_OUT_LIMIT[i]:
                            UPPER_OUT[i] = -MOTOR_OUT_LIMIT[i]

                    else:
                        LOWER_OUT[i] = MOTOR_LOWER[i].output
                        motor_out[i] = MOTOR_LOWER[i].output
                        if LOWER_OUT[i] > MOTOR_OUT_LIMIT[i]:
                            LOWER_OUT[i] = MOTOR_OUT_LIMIT[i]
                        elif LOWER_OUT[i] < -MOTOR_OUT_LIMIT[i]:
                            LOWER_OUT[i] = -MOTOR_OUT_LIMIT[i]

                    if motor_out[i] < 0 and abs(motor_out[i])>MOTOR_OUT_LIMIT[i]:
                        motor_out[i] = -MOTOR_OUT_LIMIT[i]

                    if motor_out[i] > MOTOR_OUT_LIMIT[i]:
                        motor_out[i] = MOTOR_OUT_LIMIT[i] - 1
                    motor_out[i] = - motor_out[i]
                    if motor_out[i] < 0:
                        motor_out[i] = motor_out[i]+65536
                    MOTOR_Updated[i] = True
                    MOTOR_ANGLE_MSG_OUT[i] = MOTOR_Angle[i]
                    MOTOR_SPEED_MSG_OUT[i] = MOTOR_OMEGA[i]
                    MOTOR_TORQUE_MSG_OUT[i] = torque
                    break

            if False not in MOTOR_Updated:
                if PRINT_MOTOR_INFO and printcount > 500:

                    prt_angle = " Ang: "
                    prt_spd = " Spd: "
                    prt_trq = " Trq: "
                    prt_up_out = " Up.out: "
                    prt_low_out = " Low.out: "
                    prt_control_signal = " Ctrl: "
                    prt_angle_msg = " Ang-Msg: "
                    prt_spd_msg = " Spd-Msg: "
                    prt_trq_msg = " Trq-Msg: "
                    for i in PRINT_RANGE:
                        prt_angle = prt_angle + str(i) + "[" + get_sign(MOTOR_Angle[i]) + ("%04.2f] " % (abs(MOTOR_Angle[i])))
                        prt_spd = prt_spd + str(i) + "[" + get_sign(MOTOR_OMEGA[i]) + ("%04.2f] " % (abs(MOTOR_OMEGA[i])))
                        prt_trq = prt_trq + str(i) + "[" + get_sign(MOTOR_Torque[i]) + ("%04.2f] " % (abs(MOTOR_Torque[i])))
                        prt_up_out = prt_up_out + str(i) + "[" + get_sign(MOTOR_UPPER[i].output) + ("%04d] " % (abs(MOTOR_UPPER[i].output)))
                        prt_low_out = prt_low_out + str(i) + "[" + get_sign(MOTOR_LOWER[i].output) + ("%04d] " % (abs(MOTOR_LOWER[i].output)))
                        prt_control_signal = prt_control_signal + str(i) + ("[0x%02x 0x%02x] " % ( int(int(motor_out[i])/256), int(int(motor_out[i])%(256)) ))
                        prt_angle_msg = prt_angle_msg + str(i) + "[" + get_sign(MOTOR_ANGLE_MSG_OUT[i]) + ("%04.2f] " % (abs(MOTOR_ANGLE_MSG_OUT[i])))
                        prt_spd_msg = prt_spd_msg + str(i) + "[" + get_sign(MOTOR_SPEED_MSG_OUT[i]) + ("%04.2f] " % (abs(MOTOR_SPEED_MSG_OUT[i])))
                        prt_trq_msg = prt_trq_msg + str(i) + "[" + get_sign(MOTOR_TORQUE_MSG_OUT[i]) + ("%04.2f] " % (abs(MOTOR_TORQUE_MSG_OUT[i])))
                    printing = ">>> "
                    if PRINT_Motor_Angle:
                        printing = printing + prt_angle
                    if PRINT_Motor_Speed:
                        printing = printing + prt_spd
                    if PRINT_Motor_Torque:
                        printing = printing + prt_trq
                    if PRINT_Upper_Output:
                        printing = printing + prt_up_out
                    if PRINT_Lower_Output:
                        printing = printing + prt_low_out
                    if PRINT_Control_Signal:
                        printing = printing + prt_control_signal
                    if PRINT_Angle_Massage:
                        printing = printing + prt_angle_msg
                    if PRINT_Speed_Massage:
                        printing = printing + prt_spd_msg
                    if PRINT_Torque_Massage:
                        printing = printing + prt_trq_msg
                    if PRINT_ROLLING:
                        print(printing)
                    else:
                        print("\r"+ printing)
                    printcount = 0
                else:
                    printcount = printcount + 1


                CAN_PACK = []
                for i in range(4):
                    CAN_PACK.append(int(int(motor_out[i])/256))
                    CAN_PACK.append(int(int(motor_out[i])%256))
                can_pkt = struct.pack(fmt, 0x200,8,bytes(CAN_PACK))
                sock.send(can_pkt)
                CAN_PACK = []
                for i in range(3):
                    CAN_PACK.append(int(int(motor_out[i+4])/256))
                    CAN_PACK.append(int(int(motor_out[i+4])%256))
                can_pkt = struct.pack(fmt, 0x1FF,8,bytes(CAN_PACK))

                #can_pkt = struct.pack(fmt, 0x200,8, bytes([0,0,0,0,0,0,0,0]))
                sock.send(can_pkt)
                if  1:
                    msg_content = {"Type": "MotorFeedback","Angle" : MOTOR_ANGLE_MSG_OUT, "Speed" : MOTOR_SPEED_MSG_OUT, "Torque" : MOTOR_TORQUE_MSG_OUT, "ID" : MOTOR_ID_DES, "Upper": UPPER_OUT, "Lower": LOWER_OUT}
                    #print(BSP_ERROR.info(msg_content))
                    client.publish("/MOTOR/", json.dumps(msg_content))
                    publish_real_pid()
                    mqtt_count = 0
                else:
                    mqtt_count = mqtt_count + 1
                for i in range(4):
                    MOTOR_Updated[i] = False


client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

print(BSP_ERROR.info("MQTT Interface Start Binding."))

client.connect("127.0.0.1", 1883, 60)

# Msg_thread = MsgThread()
# Msg_thread.start()
# Msg_imu_thread = Msg_imu_Thread()
# Msg_imu_thread.start()

client.loop_forever()

try:
    while True:
        time.sleep(.1)
except KeyboardInterrupt:
            print("Ctrl_C Interrupted")
            can_pkt = struct.pack(fmt, 0x200,8, bytes([0,0,0,0,0,0,0,0]))
            sock.send(can_pkt)
            can_pkt = struct.pack(fmt, 0x1FF,8, bytes([0,0,0,0,0,0,0,0]))
            sock.send(can_pkt)
            exit()

