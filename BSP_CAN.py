import socket, struct, sys, json, time, os.path, threading,math
import paho.mqtt.client as mqtt
import BSP_ERROR, BSP_PID as PID

PRINT_MOTOR_INFO = True
PRINT_ROLLING = False
PRINT_RANGE = range(4,6)

PRINT_Motor_Angle = True
PRINT_Motor_Speed = False
PRINT_Speed_Output = True
PRINT_Torque_Output = True
PRINT_Control_Signal = False
PRINT_Angle_Massage = False
PRINT_Speed_Massage = False
PRINT_Torque_Massage = False

CHASSIS_SPEED_INDEX = 500
MOTOR_ID_HEX = [0x201, 0x202, 0x203, 0x204, 0x205, 0x206, 0x207]
mono = 6

version = "01A00B " + time.ctime(os.path.getctime(os.sys.argv[0]))

CHASSIS_SPEED_SETTINGS = {"P":18, "I":0.0, "D":0.0}
CHASSIS_TORQUE_SETTINGS = {"P":0.1 ,"I":0.0, "D":0.0}

GIMBAL_YAW_SPEED_SETTINGS = {"P":5.0 ,"I":0.0, "D":0.0}
GIMBAL_YAW_TORQUE_SETTINGS = {"P":1.5 ,"I":0.0, "D":0.0}

GIMBAL_PITCH_SPEED_SETTINGS = {"P":5.0 ,"I":0.0, "D":0.0}
GIMBAL_PITCH_TORQUE_SETTINGS = {"P":1.5 ,"I":0.0, "D":0.0}

FEEDING_SPEED_SETTINGS = {"P":0.0 ,"I":0.0, "D":0.0}
FEEDING_TORQUE_SETTINGS = {"P":0.0 ,"I":0.0, "D":0.0}

MOTOR_SPEED_SETTINS = []
MOTOR_TORQUE_SETTINS = []
for i in range(4):
    MOTOR_SPEED_SETTINS.append(CHASSIS_SPEED_SETTINGS)        
    MOTOR_TORQUE_SETTINS.append(CHASSIS_TORQUE_SETTINGS)
MOTOR_SPEED_SETTINS.append(GIMBAL_YAW_SPEED_SETTINGS)
MOTOR_TORQUE_SETTINS.append(GIMBAL_YAW_TORQUE_SETTINGS)
MOTOR_SPEED_SETTINS.append(GIMBAL_PITCH_SPEED_SETTINGS)
MOTOR_TORQUE_SETTINS.append(GIMBAL_PITCH_TORQUE_SETTINGS)
MOTOR_SPEED_SETTINS.append(FEEDING_SPEED_SETTINGS)
MOTOR_TORQUE_SETTINS.append(FEEDING_TORQUE_SETTINGS)



MOTOR_SPEED = []
MOTOR_SPEED_SetPoints = [0, 0, 0, 0, 174, 174, 0]
for i in range(mono):
    MOTOR_SPEED.append(PID.PID(MOTOR_SPEED_SETTINS[i]["P"], MOTOR_SPEED_SETTINS[i]["I"], MOTOR_SPEED_SETTINS[i]["D"]))
    MOTOR_SPEED[i].SetPoint=MOTOR_SPEED_SetPoints[i]
    MOTOR_SPEED[i].setSampleTime(0.01)



MOTOR_TORQUE = []
MOTOR_TORQUE_SetPoints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
for i in range(mono):
    MOTOR_TORQUE.append(PID.PID(MOTOR_TORQUE_SETTINS[i]["P"], MOTOR_TORQUE_SETTINS[i]["I"], MOTOR_TORQUE_SETTINS[i]["D"]))
    MOTOR_TORQUE[i].SetPoint=MOTOR_TORQUE_SetPoints[i]
    MOTOR_TORQUE[i].setSampleTime(0.001)


print(BSP_ERROR.access("BSP CAN START RUNNING, Version:" + version))
fmt = "<IB3x8s" #Regex for CAN Protocol

print(BSP_ERROR.info("Socket CAN Interface Start Binding."))
sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) #Socket CAN
interface = "can0"

try:
    sock.bind((interface,))
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
        # MOTOR_Updated.append(False)
        MOTOR_ID_DES.append(i)
        MOTOR_ANGLE_MSG_OUT.append(0.0)
        MOTOR_SPEED_MSG_OUT.append(0.0)
        MOTOR_TORQUE_MSG_OUT.append(0.0)
    chs_data = [[],[],[],[]]

    mqtt_count = 0
    phi_count = [0,0,0,0,0,0,0]

    while 1:
        can_pkt = sock.recv(16)
        can_id, length, data = struct.unpack(fmt, can_pkt)
        can_id &= socket.CAN_EFF_MASK


        data = data[:length]

        if can_id in MOTOR_ID_HEX:

            torque = data[4]*256+data[5]
            if torque >= 2**15:
                torque = torque-2**16
            speed = data[2]*256+data[3]
            if speed >= 2**15:
                speed = speed-2**16

            for i in range(mono):
                if can_id == MOTOR_ID_HEX[i] :
                    if phi_count[i] > 10: #reduce the speed of phi
                        MOTOR_Now[i] = (360.0)/(8191)*(data[0]*256+data[1])
                        MOTOR_Phi[i] = MOTOR_Now[i] - MOTOR_Angle[i]
                        if MOTOR_Phi[i] > 180:
                            MOTOR_Phi[i] = MOTOR_Phi[i] - 360
                        elif MOTOR_Phi[i] < -180:
                            MOTOR_Phi[i] = MOTOR_Phi[i] + 360
                        MOTOR_Angle[i] = MOTOR_Now[i]

                        if i in range(4):
                            MOTOR_SPEED[i].update(MOTOR_Phi[i]*10)
                        elif i in range(4,6):
                            MOTOR_SPEED[i].update(MOTOR_Now[i])
                        MOTOR_TORQUE[i].SetPoint = MOTOR_SPEED[i].output
                        phi_count[i] = 0
                    else:
                        phi_count[i] = phi_count[i] + 1

                    MOTOR_TORQUE[i].update(torque)
                    motor_out[i] = MOTOR_TORQUE[i].output

                    if motor_out[i] < 0 and abs(motor_out[i])>2**15:
                        motor_out[i] = -2**15

                    if motor_out[i] > 2**15:
                        motor_out[i] = 2**15-1

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
                        prt_angle = prt_angle + str(i) + "[" + get_sign(MOTOR_Angle[i]) + ("%04.2f] " % (abs(MOTOR_Angle[i])))
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

                        print("\r"+ printing, end="")


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
                if  mqtt_count > 50:
                    msg_content = {"Type": "MotorFeedback","Angle" : MOTOR_ANGLE_MSG_OUT, "Speed" : MOTOR_SPEED_MSG_OUT, "Torque" : MOTOR_TORQUE_MSG_OUT, "ID" : MOTOR_ID_DES}
                    #print(BSP_ERROR.info(msg_content))
                    client.publish("/MOTOR/", json.dumps(msg_content))
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

