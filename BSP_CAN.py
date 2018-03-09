import socket, struct, sys, json, time, os.path, threading,math
from bitstring import Bits
import paho.mqtt.client as mqtt
import BSP_ERROR, BSP_PID as PID

version = "00A01B " + time.ctime(os.path.getctime(os.sys.argv[0]))

CHASSIS_SPEED_SETTINS = {"P":24.0, "I":5.0, "D":1.0}
Chassis_1_Speed = PID.PID(CHASSIS_SPEED_SETTINS["P"], CHASSIS_SPEED_SETTINS["I"], CHASSIS_SPEED_SETTINS["D"])
Chassis_1_Speed.SetPoint=-500
Chassis_1_Speed.setSampleTime(0.001)

CHASSIS_TORQUE_SETTINS = {"P":0.1, "I":0.1, "D":0.0002}


Chassis_1_Torque = PID.PID(CHASSIS_TORQUE_SETTINS["P"], CHASSIS_TORQUE_SETTINS["I"], CHASSIS_TORQUE_SETTINS["D"])
Chassis_1_Torque.SetPoint=0.0
Chassis_1_Torque.setSampleTime(0.001)

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

def CAN_RCV_LOOP():
    Chassis_1_Angle = 0.0
    while 1:
        can_pkt = sock.recv(16)
        can_id, length, data = struct.unpack(fmt, can_pkt)
        can_id &= socket.CAN_EFF_MASK
        data = data[:length]

        if can_id == 0x204:
            torque = data[4]*256+data[5]
            if torque >= 2**15:
                torque = -2**16+torque
            speed = data[2]*256+data[3]
            if speed >= 2**15:
                speed = (speed-2**16)

            Chassis_Phi = (360.0)/(8191)*(data[0]*256+data[1]) - Chassis_1_Angle


            Chassis_1_Angle = (360.0)/(8191)*(data[0]*256+data[1])

            if abs(Chassis_Phi) > 100:
                continue

            Chassis_1_Speed.update(Chassis_Phi*100)
            Chassis_1_Torque.SetPoint = Chassis_1_Speed.output

            Chassis_1_Torque.update(torque)
            chs1_out = Chassis_1_Torque.output

            if chs1_out < 0 and abs(chs1_out)>2**15:
                chs1_out = -2**15

            if chs1_out > 2**15:
                chs1_out = 2**15-1

            if chs1_out < 0:
                chs1_out = chs1_out+65536




            print("Speed: %06d Phi: %06f Torque: %06d Data: 0x%02x 0x%02x" % (Chassis_1_Speed.output, Chassis_Phi*100 ,Chassis_1_Torque.output  ,int(int(chs1_out)/256), int(int(chs1_out)%(256))))
            can_pkt = struct.pack(fmt, 0x200,8,bytes([0,0,0,0,0,0,int(int(chs1_out)/256),int(int(chs1_out)%256)]))
            sock.send(can_pkt)
            msg_content = {"Type": "MotorFeedback","Angle" : (360.0)/(8191)*(data[0]*256+data[1]), "Speed" : speed, "Torque" : torque, "ID" : int(can_id)-0x200}
            print(BSP_ERROR.info(msg_content))
            #mqtt.publish("/MOTOR/", json.dumps(msg_content))


client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

print(BSP_ERROR.info("MQTT Interface Start Binding."))

client.connect("127.0.0.1", 1883, 60)
client.loop_forever()
