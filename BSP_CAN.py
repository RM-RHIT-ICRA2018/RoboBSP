import socket, struct, sys, json, time, os.path, threading, math
import serial, select, copy, pdb, pyvesc as esc
import paho.mqtt.client as mqtt
import BSP_ERROR, BSP_PID as PID

PRINT_MOTOR_INFO = 1
PRINT_ROLLING = False
PRINT_RANGE = [6]

PRINT_Motor_Angle = True
PRINT_Motor_Speed = False
PRINT_Speed_Output = True
PRINT_Torque_Output = True
PRINT_Control_Signal = False
PRINT_Angle_Massage = False
PRINT_Speed_Massage = False
PRINT_Torque_Massage = False


CHASSIS_SPEED_INDEX = 1000
MOTOR_ID_HEX = [0x201, 0x202, 0x203, 0x204, 0x205, 0x206, 0x207]
mono = 7

init = []
for i in range(mono):
    init.append(True)

SHOTTER_MOTOR_REVERSE = False

version = "01A00B " + time.ctime(os.path.getctime(os.sys.argv[0]))

PID_Items = ["Chassis_Speed", "Chassis_Torque","Yaw_Speed","Yaw_Torque","Pitch_Speed","Pitch_Torque","Feeding_Speed","Feeding_Torque"]
PID_Num = len(PID_Items)

SERIAL_COMM = []

PID_SETTINGS_SET = []
PID_SETTINGS_SET.append({"P":18, "I":0.0, "D":0.0})             #Chassis_Speed
PID_SETTINGS_SET.append({"P":0.1 ,"I":0.0, "D":0.0})            #Chassis_Torque

PID_SETTINGS_SET.append({"P":-60.0 ,"I":-8.0, "D":-2.0})        #Yaw_Speed
PID_SETTINGS_SET.append({"P":0.1 ,"I":0.0, "D":0.0})            #Yaw_Torque

PID_SETTINGS_SET.append({"P":0.0 ,"I":0.0 ,"D":0.0})      #Pitch_Speed #{"P":-95.0 ,"I":-125.0 ,"D":-4.5}
PID_SETTINGS_SET.append({"P":0.2 ,"I":0.0, "D":0.0})            #Pitch_Torque

PID_SETTINGS_SET.append({"P":5.0 ,"I":5, "D":0.02})             #Feeding_Speed
PID_SETTINGS_SET.append({"P":0.5 ,"I":0.0, "D":0.0})            #Feeding_Torque

PID_SETTINGS_REAL = copy.deepcopy(PID_SETTINGS_SET)

MOTOR_SPEED_SETTINS = []
MOTOR_TORQUE_SETTINS = []

for i in range(4):
    MOTOR_SPEED_SETTINS.append(PID_SETTINGS_REAL[0])
    MOTOR_TORQUE_SETTINS.append(PID_SETTINGS_REAL[1])
MOTOR_SPEED_SETTINS.append(PID_SETTINGS_REAL[2])
MOTOR_TORQUE_SETTINS.append(PID_SETTINGS_REAL[3])
MOTOR_SPEED_SETTINS.append(PID_SETTINGS_REAL[4])
MOTOR_TORQUE_SETTINS.append(PID_SETTINGS_REAL[5])
MOTOR_SPEED_SETTINS.append(PID_SETTINGS_REAL[6])
MOTOR_TORQUE_SETTINS.append(PID_SETTINGS_REAL[7])


MOTOR_SPEED = []
MOTOR_SPEED_SetPoints = [0, 0, 0, 0, 174, 174, 0]
for i in range(mono):
    MOTOR_SPEED.append(PID.PID(MOTOR_SPEED_SETTINS[i]["P"], MOTOR_SPEED_SETTINS[i]["I"], MOTOR_SPEED_SETTINS[i]["D"]))
    MOTOR_SPEED[i].SetPoint=MOTOR_SPEED_SetPoints[i]
    MOTOR_SPEED[i].setSampleTime(0.01)



MOTOR_TORQUE = []
MOTOR_TORQUE_SetPoints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
MOTOR_TORQUE_LIMIT = [32768, 32768, 32768, 32768, 20000, 20000, 32768]

for i in MOTOR_TORQUE_LIMIT:
    assert i <= 2**15 # THE MAX ABC_TORQUE IS 2**15

for i in range(mono):
    MOTOR_TORQUE.append(PID.PID(MOTOR_TORQUE_SETTINS[i]["P"], MOTOR_TORQUE_SETTINS[i]["I"], MOTOR_TORQUE_SETTINS[i]["D"]))
    MOTOR_TORQUE[i].SetPoint=MOTOR_TORQUE_SetPoints[i]
    MOTOR_TORQUE[i].setSampleTime(0.001)

print(BSP_ERROR.access("BSP CAN START RUNNING, Version:" + version))
fmt = "<IB3x8s" #Regex for CAN Protocol

print(BSP_ERROR.info("Socket CAN Interface Start Binding."))
sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) #Socket CAN
interface = "can0"

def update_PID():
    for i in range(mono):
        MOTOR_SPEED[i].clear()
        MOTOR_SPEED[i].setKp(MOTOR_SPEED_SETTINS[i]["P"])
        MOTOR_SPEED[i].setKi(MOTOR_SPEED_SETTINS[i]["I"])
        MOTOR_SPEED[i].setKd(MOTOR_SPEED_SETTINS[i]["D"])
        MOTOR_TORQUE[i].clear()
        MOTOR_TORQUE[i].setKp(MOTOR_TORQUE_SETTINS[i]["P"])
        MOTOR_TORQUE[i].setKi(MOTOR_TORQUE_SETTINS[i]["I"])
        MOTOR_TORQUE[i].setKd(MOTOR_TORQUE_SETTINS[i]["D"])


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

def on_connect(client, userdata, flags, rc):
    print(BSP_ERROR.notice("MQTT Interface Bind Success."))
    client.subscribe("/CANBUS/#")
    client.subscribe("/REMOTE/#")
    print(BSP_ERROR.notice("MQTT Subscribe Success, Topic: /CANBUS/#, Start Receiving CAN Messages."))
    t = threading.Thread(target = CAN_RCV_LOOP)
    t.start()

def on_message(client, userdata, msg):
    print(BSP_ERROR.info("Topic: "+ msg.topic + " Payload: " + msg.payload.decode("utf-8")))
    payload = json.loads(msg.payload.decode("utf-8"))
    # if payload["Type"] == "MotorTye":
    #     can_pkt = struct.pack(fmt, int(payload.ID),8,bytes(payload.Torques))
    #     sock.send(can_pkt)
    #     print(BSP_ERROR.info("SocketCAN Package Send"))
    if msg.topic == "/REMOTE/":
        Robot_X = payload["XSpeed"]
        Robot_Y = payload["YSpeed"]
        Robot_Phi = payload["PhiSpeed"]
        MOTOR_Remote = [-Robot_X+Robot_Y+Robot_Phi, Robot_X+Robot_Y+Robot_Phi, Robot_X-Robot_Y+Robot_Phi, -Robot_X-Robot_Y+Robot_Phi]
        for i in range(4):
            MOTOR_SPEED[i].SetPoint = MOTOR_Remote[i]*CHASSIS_SPEED_INDEX
        return
    elif msg.topic == "/PID_REMOTE/" :
        Ps = payload.get("Ps")
        Is = payload.get("Is")
        Ds = payload.get("Ds")
        for i in range(PID_Num):
            PID_SETTINGS_REAL[i]["P"] = Ps[i]
            PID_SETTINGS_REAL[i]["I"] = Is[i]
            PID_SETTINGS_REAL[i]["D"] = Ds[i]
        update_PID()
        publish_real_pid()

    if msg.topic == "/REMOTE/EXP":
        #MOTOR_SPEED[4].SetPoint = payload["YawAngle"]
        #MOTOR_SPEED[5].SetPoint = payload["PitchAngle"]
        MOTOR_SPEED[6].SetPoint = MOTOR_SPEED[6].SetPoint + payload["Pos"]

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
        Ps.append(MOTOR_SPEED[3+i].getP())
        Is.append(MOTOR_SPEED[3+i].getI())
        Ds.append(MOTOR_SPEED[3+i].getD())

        Ps.append(MOTOR_TORQUE[3+i].getP())
        Is.append(MOTOR_TORQUE[3+i].getI())
        Ds.append(MOTOR_TORQUE[3+i].getD())
    agree = compare_pid()
    pid_msg = {"Ps":Ps, "Is":Is, "Ds":Ds, "Agree": agree}
    client.publish("/PID_FEEDBACK/", json.dumps(pid_msg))




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

    for i in range(7):
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

    mqtt_count = 0
    phi_count = [0,0,0,0,0,0,0]

    while 1:
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

            for i in range(mono):
                if can_id == MOTOR_ID_HEX[i] :
                    if 1:#phi_count[i] > 10: #reduce the speed of phi
                        MOTOR_Now[i] = (360.0)/(8191)*(data[0]*256+data[1])
                        if init[i]:
                            MOTOR_Angle[i] = MOTOR_Now[i]
                            init[i] = False
                        MOTOR_Phi[i] = MOTOR_Now[i] - MOTOR_Angle[i]
                        if MOTOR_Phi[i] > 180:
                            MOTOR_Phi[i] = MOTOR_Phi[i] - 360
                        elif MOTOR_Phi[i] < -180:
                            MOTOR_Phi[i] = MOTOR_Phi[i] + 360
                        MOTOR_Angle[i] = MOTOR_Now[i]
                        MOTOR_Total[i] = MOTOR_Total[i] + MOTOR_Phi[i]

                        if i in range(4):
                            MOTOR_SPEED[i].update(MOTOR_Phi[i]*10)
                        elif i in range(4,6):
                            MOTOR_SPEED[i].update(MOTOR_Angle[i])
                        elif i in range(6,7):
                            MOTOR_SPEED[i].update(MOTOR_Total[i])

                        MOTOR_TORQUE[i].SetPoint = MOTOR_SPEED[i].output
                        phi_count[i] = 0
                    else:
                        phi_count[i] = phi_count[i] + 1

                    MOTOR_TORQUE[i].update(torque)
                    motor_out[i] = MOTOR_TORQUE[i].output

                    if i == 4:
                        motor_out[4] = MOTOR_SPEED[4].output
                    if i == 5:
                        motor_out[5] = MOTOR_SPEED[5].output

                    if motor_out[i] < 0 and abs(motor_out[i])>MOTOR_TORQUE_LIMIT[i]:
                        motor_out[i] = -MOTOR_TORQUE_LIMIT[i]

                    if motor_out[i] > MOTOR_TORQUE_LIMIT[i]:
                        motor_out[i] = MOTOR_TORQUE_LIMIT[i] - 1

                    if motor_out[i] < 0:
                        motor_out[i] = motor_out[i]+65536
                    MOTOR_Updated[i] = True
                    MOTOR_ANGLE_MSG_OUT[i] = (360.0)/(8191)*(data[0]*256+data[1])
                    MOTOR_SPEED_MSG_OUT[i] = speed
                    MOTOR_TORQUE_MSG_OUT[i] = torque
                    break

            if False not in MOTOR_Updated:
                if PRINT_MOTOR_INFO:

                    prt_angle = " Ang: "
                    prt_spd = " Spd: "
                    prt_spd_out = " Spd.out: "
                    prt_trq_out = " Trq.out: "
                    prt_control_signal = " Ctrl: "
                    prt_angle_msg = " Ang-Msg: "
                    prt_spd_msg = " Spd-Msg: "
                    prt_trq_msg = " Trq-Msg: "
                    for i in PRINT_RANGE:
                        prt_angle = prt_angle + str(i) + "[" + get_sign(MOTOR_Angle[i]) + ("%04.2f] " % (abs(MOTOR_Total[i])))
                        prt_spd = prt_spd + str(i) + "[" + get_sign(MOTOR_Phi[i]*100) + ("%04.2f] " % (abs(MOTOR_Phi[i]*100)))
                        prt_spd_out = prt_spd_out + str(i) + "[" + get_sign(MOTOR_SPEED[i].output) + ("%04d] " % (abs(MOTOR_SPEED[i].output)))
                        prt_trq_out = prt_trq_out + str(i) + "[" + get_sign(MOTOR_TORQUE[i].output) + ("%04d] " % (abs(MOTOR_TORQUE[i].output)))
                        prt_control_signal = prt_control_signal + str(i) + ("[0x%02x 0x%02x] " % ( int(int(motor_out[i])/256), int(int(motor_out[i])%(256)) ))
                        prt_angle_msg = prt_angle_msg + str(i) + "[" + get_sign(MOTOR_ANGLE_MSG_OUT[i]) + ("%04.2f] " % (abs(MOTOR_ANGLE_MSG_OUT[i])))
                        prt_spd_msg = prt_spd_msg + str(i) + "[" + get_sign(MOTOR_SPEED_MSG_OUT[i]) + ("%04.2f] " % (abs(MOTOR_SPEED_MSG_OUT[i])))
                        prt_trq_msg = prt_trq_msg + str(i) + "[" + get_sign(MOTOR_TORQUE_MSG_OUT[i]) + ("%04.2f] " % (abs(MOTOR_TORQUE_MSG_OUT[i])))
                    printing = ">>> "
                    if PRINT_Motor_Angle:
                        printing = printing + prt_angle
                    if PRINT_Motor_Speed:
                        printing = printing + prt_spd
                    if PRINT_Speed_Output:
                        printing = printing + prt_spd_out
                    if PRINT_Torque_Output:
                        printing = printing + prt_trq_out
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
                if  mqtt_count > 25:
                    msg_content = {"Type": "MotorFeedback","Angle" : MOTOR_ANGLE_MSG_OUT, "Speed" : MOTOR_SPEED_MSG_OUT, "Torque" : MOTOR_TORQUE_MSG_OUT, "ID" : MOTOR_ID_DES}
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

