import socket, struct, sys, json, time, os.path, thread
import paho.mqtt.client as mqtt
import BSP_ERROR

version = "00A01B " + time.ctime(os.path.getctime(os.sys.argv[0]))


print(access("BSP CAN START RUNNING, Version:" + version))
fmt = "<IB3x8s" #Regex for CAN Protocol

print(info("Socket CAN Interface Start Binding."))
sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) #Socket CAN
interface = "can0"

try:
    sock.bind((interface,))
except OSError:
    print(fail("Could not bind to interface '%s'\n" % interface))
    exit()

print(notice("Socker CAN Interface Binding Success."))

def on_connect(client, userdata, flags, rc):
    print(notice("MQTT Interface Bind Success."))
    client.subscribe("/CANBUS/#")
    print(notice("MQTT Subscribe Success, Topic: /CANBUS/#, Start Receiving CAN Messages."))
    thread.start_new_thread(CAN_RCV_LOOP, (""))

def on_message(client, userdata, msg):
    print(info("Topic: "+ msg.topic + " Payload: " + msg.payload))
    payload = json.decoder(msg.payload)
    if 0 == cmp(payload.Type,"MotorTye"):
        can_pkt = struct.pack(fmt, int(payload.ID),8,bytes(payload.Torques))
        sock.send(can_pkt)
        print(info("SocketCAN Package Send"))

def CAN_RCV_LOOP(args):
    while 1:
        can_pkt = sock.recv(16)
        can_id, length, data = struct.unpack(fmt, can_pkt)
        can_id &= socket.CAN_EFF_MASK
        data = data[:length]

        if can_id >= 0x200 and can_id <= 0x208:
            torque = data[4]*256+data[5]
            if torque >= 2**15:
                torque = -(2**16-torque)
            speed = data[2]*256+data[3]
            if speed >= 2**15:
                speed = -(2**16-speed)
            msg_content = {"Type": "MotorFeedback","Angle" : (360.0)/(8191)*(data[0]*256+data[1]),
    "Speed" : speed, "Torque" : torque, "ID" : int(can_id)-0x200}
            print(info(msg_content))
            mqtt.publish("/MOTOR/", json.encoder(msg_content))
            

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

print(info("MQTT Interface Start Binding."))

client.connect("127.0.0.1", 1883, 60)
client.loop_start()