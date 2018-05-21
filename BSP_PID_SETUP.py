import json, time, threading, math
import BSP_ERROR, BSP_PID as PID
import BSP_ROBOT_CONFIG as ROB
import copy
import paho.mqtt.client as mqtt

NAME = "BSP_PID_SETUP"
start = False

Ps = [ 180,   0,  20,  15,  20,  15,   0,   0, 0.5, 0.5,-0.5,   0,   0]
Is = [  30,   0,   0,  30,   0,  30,   0,   0,   0,   0,   0,   0,   0]
Ds = [   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0]

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    client.subscribe("/SYS/#")
    client.subscribe("/PID_FEEDBACK/#")

def on_message(client, userdata, msg):
    global start
    global Ps
    global Is
    global Ds
    if msg.topic == "/SYS/INIT/STR":
        start = True
        client.publish("/PID_REMOTE/", json.dumps({"Ps": Ps, "Is": Is, "Ds": Ds}))
    elif msg.topic == "/SYS/INIT/END":
        start = False
        client.publish("/PID_REMOTE/", json.dumps({"Ps": [0,0,0,0,0,0,0,0,0,0,0,0,0], "Is": [0,0,0,0,0,0,0,0,0,0,0,0,0], "Ds": [0,0,0,0,0,0,0,0,0,0,0,0,0]}))
        client.publish("/FAILSAFE/", json.dumps({"Type": "ForceShutDown"}))







client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

print(BSP_ERROR.info("MQTT Interface Start Binding."))

client.connect("127.0.0.1", 1883, 60)

# Pub_thread = PubThread()
# Pub_thread.start()

client.loop_start()

client.publish("/SYS/APP/STR", json.dumps({"Name": NAME, "Time": time.time()}))

while True:
    time.sleep(1)
    client.publish("/SYS/APP/HBT", json.dumps({"Name": NAME, "Time": time.time()}))
    if start:
        client.publish("/PID_REMOTE/", json.dumps({"Ps": Ps, "Is": Is, "Ds": Ds}))
    else:
        client.publish("/PID_REMOTE/", json.dumps({"Ps": [0,0,0,0,0,0,0,0,0,0,0,0,0], "Is": [0,0,0,0,0,0,0,0,0,0,0,0,0], "Ds": [0,0,0,0,0,0,0,0,0,0,0,0,0]}))
