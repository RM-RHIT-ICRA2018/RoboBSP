import socket, struct, sys, json, time, os.path, threading,math
from bitstring import Bits
import paho.mqtt.client as mqtt
import BSP_ERROR, BSP_PID as PID

CHASSIS_SPEED_INDEX = 500
CHASSIS_ID = [0x201, 0x202, 0x203, 0x204]

version = "01A00B " + time.ctime(os.path.getctime(os.sys.argv[0]))

CHASSIS_SPEED_SETTINS = {"P":10, "I":0, "D":0}
Chassis_Speed = []
Chassis_Speed_SetPoints = [0, 0, 0, 0]
for i in range(4):
    Chassis_Speed.append(PID.PID(CHASSIS_SPEED_SETTINS["P"], CHASSIS_SPEED_SETTINS["I"], CHASSIS_SPEED_SETTINS["D"]))
    Chassis_Speed[i].SetPoint=Chassis_Speed_SetPoints[i]
    Chassis_Speed[i].setSampleTime(0.001)

# Chassis_1_Speed = PID.PID(CHASSIS_SPEED_SETTINS["P"], CHASSIS_SPEED_SETTINS["I"], CHASSIS_SPEED_SETTINS["D"])
# Chassis_1_Speed.SetPoint=-500
# Chassis_1_Speed.setSampleTime(0.001)

CHASSIS_TORQUE_SETTINS = {"P":0.1, "I":0, "D":0}
Chassis_Torque = []
Chassis_Torque_SetPoints = [0.0, 0.0, 0.0, 0.0]
for i in range(4):
    Chassis_Torque.append(PID.PID(CHASSIS_TORQUE_SETTINS["P"], CHASSIS_TORQUE_SETTINS["I"], CHASSIS_TORQUE_SETTINS["D"]))
    Chassis_Torque[i].SetPoint=Chassis_Torque_SetPoints[i]
    Chassis_Torque[i].setSampleTime(0.001)


# Chassis_1_Torque = PID.PID(CHASSIS_TORQUE_SETTINS["P"], CHASSIS_TORQUE_SETTINS["I"], CHASSIS_TORQUE_SETTINS["D"])
# Chassis_1_Torque.SetPoint=0.0
# Chassis_1_Torque.setSampleTime(0.001)

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
    print(BSP_ERROR.notice("MQTT Subscribe Success, Topic: /CANBUS/#, Start Receiving CAN Messages."))
    t = threading.Thread(target = CAN_RCV_LOOP)
    t.start()

def on_message(client, userdata, msg):
    print(BSP_ERROR.info("Topic: "+ msg.topic + " Payload: " + msg.payload))
    payload = json.loads(msg.payload)
    if payload.Type == "MotorTye":
        can_pkt = struct.pack(fmt, int(payload.ID),8,bytes(payload.Torques))
        sock.send(can_pkt)
        print(BSP_ERROR.info("SocketCAN Package Send"))
    elif msg.topic == "/REMOTE/":
        Robot_X = payload.XSpeed
        Robot_Y = payload.YSPeed
        Robot_Phi = payload.PhiSpeed
        Chassis_Remote = [-Robot_X+Robot_Y+Robot_Phi, Robot_X+Robot_Y+Robot_Phi, Robot_X-Robot_Y+Robot_Phi, -Robot_X-Robot_Y+Robot_Phi]
        for i in range(4):
            Chassis_Speed[i].SetPoint = Chassis_Remote[i]*CHASSIS_SPEED_INDEX

def CAN_RCV_LOOP():
    Chassis_Angle = [0.0, 0.0, 0.0, 0.0]
    Chassis_Phi = [0.0, 0.0, 0.0, 0.0]
    Chassis_Now = [0.0, 0.0, 0.0, 0.0]
    chs_out = [0.0, 0.0, 0.0, 0.0]
    Chassis_Updated = [False, False, False, False]
    Motor_ID = [0,1,2,3,4,5,6]
    Motor_Angle = []
    Motor_Speed = []
    Motor_Torque = []
    for i in range(7):
        Motor_Angle.append(0.0)
        Motor_Speed.append(0.0)
        Motor_Torque.append(0.0)
    chs_data = [[],[],[],[]]

    # Chassis_1_Angle = 0.0
    while 1:
        can_pkt = sock.recv(16)
        can_id, length, data = struct.unpack(fmt, can_pkt)
        can_id &= socket.CAN_EFF_MASK


        data = data[:length]


        if can_id in CHASSIS_ID:

            torque = data[4]*256+data[5]
            if torque >= 2**15:
                torque = -2**16+torque
            speed = data[2]*256+data[3]
            if speed >= 2**15:
                speed = (speed-2**16)

            for i in range(4):
                if can_id == CHASSIS_ID[i] :
                    Chassis_Now[i] = (360.0)/(8191)*(data[0]*256+data[1])
                    Chassis_Phi[i] = Chassis_Now[i] - Chassis_Angle[i]
                    if Chassis_Phi[i] > 180:
                        Chassis_Phi[i] = Chassis_Phi[i] - 360
                    elif Chassis_Phi[i] < -180:
                        Chassis_Phi[i] = Chassis_Phi[i] + 360
                    Chassis_Angle[i] = Chassis_Now[i]


                    # Chassis_Phi = (360.0)/(8191)*(data[0]*256+data[1]) - Chassis_1_Angle


                    # Chassis_1_Angle = (360.0)/(8191)*(data[0]*256+data[1])

                    # if abs(Chassis_Phi) > 100:
                    #     continue
                    #print("Phi:%06f Angle:%06f" % (Chassis_Phi[0], Chassis_Angle[0]))
                    Chassis_Speed[i].update(Chassis_Phi[i]*100)
                    Chassis_Torque[i].SetPoint = Chassis_Speed[i].output
                    #print("S: ", end="")
                    #for j in range(4) :
                    #    print(str(Chassis_Speed[j].output),end="")
                    #print()
                    Chassis_Torque[i].update(torque)
                    chs_out[i] = Chassis_Torque[i].output
                    if i==0:
                        print("\rPhi:%06f Angle:%06f Speed.out:%06f ChsOut:%06f" % (Chassis_Phi[i], Chassis_Angle[i], Chassis_Speed[i].output, chs_out[i]), end="")
                    if chs_out[i] < 0 and abs(chs_out[i])>2**15:
                        chs_out[i] = -2**15

                    if chs_out[i] > 2**15:
                        chs_out[i] = 2**15-1

                    if chs_out[i] < 0:
                        chs_out[i] = chs_out[i]+65536
                    Chassis_Updated[i] = True
                    Motor_Angle[i] = (360.0)/(8191)*(data[0]*256+data[1])
                    Motor_Speed[i] = speed
                    Motor_Torque[i] = torque
                    break


            # Chassis_1_Speed.update(Chassis_Phi*100)
            # Chassis_1_Torque.SetPoint = Chassis_1_Speed.output

            # Chassis_1_Torque.update(torque)
            # chs1_out = Chassis_1_Torque.output

            # if chs1_out < 0 and abs(chs1_out)>2**15:
            #     chs1_out = -2**15

            # if chs1_out > 2**15:
            #     chs1_out = 2**15-1

            # if chs1_out < 0:
            #     chs1_out = chs1_out+65536

            if False not in Chassis_Updated:
                prt_spd = "Spd: 0[%06d] 1[%06d] 2=[%06d] 3=[%06d]  " % (Chassis_Speed[0].output, Chassis_Speed[1].output, Chassis_Speed[2].output, Chassis_Speed[3].output)
                prt_phi = "Phi: 0[%06f] 1[%06f] 2=[%06f] 3=[%06f]  " % (Chassis_Phi[0]*100, Chassis_Phi[1]*100, Chassis_Phi[2]*100, Chassis_Phi[3]*100)
                prt_trq = "Trq: 0[%06d] 1[%06d] 2=[%06d] 3=[%06d]  " % (Chassis_Torque[0].output, Chassis_Torque[1].output, Chassis_Torque[2].output, Chassis_Torque[3].output)
                prt_dt = "Data: 0[0x%02x 0x%02x] 1[0x%02x 0x%02x] 2=[0x%02x 0x%02x] 3=[0x%02x 0x%02x]  " % (int(int(chs_out[0])/256), int(int(chs_out[0])%(256)), int(int(chs_out[1])/256), int(int(chs_out[1])%(256)), int(int(chs_out[2])/256), int(int(chs_out[2])%(256)), int(int(chs_out[3])/256), int(int(chs_out[3])%(256)))

#                print(prt_spd + prt_phi + prt_trq + prt_dt)

                #print("Spd: %06d Phi: %06f Trq: %06d Data: 0x%02x 0x%02x" % (Chassis_1_Speed.output, Chassis_Phi*100,Chassis_1_Torque.output  ,int(int(chs1_out)/256), int(int(chs1_out)%(255))))
                #print(str(chs_out[0])+" "+str(chs_out[1])+" "+str(chs_out[2])+" "+str(chs_out[3]))
                can_pkt = struct.pack(fmt, 0x200,8,bytes([int(int(chs_out[0])/256),int(int(chs_out[0])%256), int(int(chs_out[1])/256),int(int(chs_out[1])%256), int(int(chs_out[2])/256),int(int(chs_out[2])%256), int(int(chs_out[3])/256),int(int(chs_out[3])%256)]))
                #can_pkt = struct.pack(fmt, 0x200,8, bytes([0,0,0,0,0,0,0,0]))
                sock.send(can_pkt)
                msg_content = {"Type": "MotorFeedback","Angle" : Motor_Angle, "Speed" : Motor_Speed, "Torque" : Motor_Torque, "ID" : Motor_ID}
                #print(BSP_ERROR.info(msg_content))
                #mqtt.publish("/MOTOR/", json.dumps(msg_content))
                Chassis_Updated = [False, False, False, False]


client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

print(BSP_ERROR.info("MQTT Interface Start Binding."))

client.connect("127.0.0.1", 1883, 60)
client.loop_forever()
