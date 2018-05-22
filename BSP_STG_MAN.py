import socket, struct, sys, json, time, os.path, threading, math, random
import json, time, threading, math
import BSP_ERROR, BSP_PID as PID
import BSP_ROBOT_CONFIG as ROB
import copy
import paho.mqtt.client as mqtt

NAME = "BSP_STR_MAN"
start = False

current_point = [0,0,0]
start_point = [0,0,0]
start_initialized = [False,False,False]
robot_ready = False
step = 0
stand_by = False

points = [[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0]]

def at_point(p1, p2):
    for i in range(3):
        error = p1[i] - p2[i]
        if i == 2:
            if error > 180:
                error = error - 360
            elif error < -180:
                error = error + 360
        if abs(error) > point_threshold[i]:
            return False
    return True


def if_ready():
    global stand_by
    global start_initialized
    if not stand_by:
        for i in range(3):
            if not start_initialized[i]:
                return False
        print("Initialization complete, start_point: [ %f, %f, %f ], current_point: [ %f, %f, %f]")
    stand_by = True
    return at_point(current_point, start_point)

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    client.subscribe("/SYS/#")
    client.subscribe("/UWB/POS")
    client.subscribe("/CHASSIS/AHRS/ALIG")

def on_message(client, userdata, msg):
    global start
    global Ps
    global Is
    global Ds
    if msg.topic == "/SYS/INIT/STR":
        time.sleep(1)
        start = True
    elif msg.topic == "/SYS/INIT/END":
        start = False
    elif msg.topic == "/UWB/POS":
        if start_initialized[0]:
            current_point[0] = payload["posX"]
        else:
            start_point[0] = payload["posX"]
            points[0][0] = payload["posX"]
            start_initialized[0] = True
        if start_initialized[1]:
            current_point[1] = payload["posY"]
        else:
            start_point[1] = payload["posY"]
            points[0][1] = payload["posY"]
            start_initialized[1] = True
        robot_ready = if_ready()
        
    elif msg.topic == "/CHASSIS/AHRS/ALIG":
        if start_initialized[2]:
            current_point[2] = payload["Yaw"]
            robot_ready = if_ready()
        else:
            start_point[2] = payload["Yaw"]
            points[0][2] = payload["Yaw"]
            start_initialized[2] = True

def navigation()
    if start:
        if step>=len(points): return
        target_point = points[step]
        while not at_point(target_point, current_point):
            time.sleep(0.05)
            client.publish("/CHASSIS/SET", json.dumps({"Type": "position", "XSet": target_point[0], "YSet": target_point[1], "PhiSet": target_point[2]}))
            client.publish("/GIMBAL/TO", json.dumps({"Yaw": target_point[3], "Pitch": target_point[4]}))
        step = step + 1
    

class NaviThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)

    def run(self):
        while 1:
            navigation()




client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

print(BSP_ERROR.info("MQTT Interface Start Binding."))

client.connect("127.0.0.1", 1883, 60)

Navi_thread = NaviThread()
Navi_thread.start()

client.loop_start()

client.publish("/SYS/APP/STR", json.dumps({"Name": NAME, "Time": time.time()}))

while True:
    time.sleep(1)
    client.publish("/SYS/APP/HBT", json.dumps({"Name": NAME, "Time": time.time()}))
