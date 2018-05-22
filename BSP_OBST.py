import socket, struct, sys, json, time, os.path, threading, math, random
import serial, select, copy, pdb, pyvesc as esc
import paho.mqtt.client as mqtt
from rplidar import RPLidar

NAME = "OBST"
CHASSIS_ANGLE = 0.0
COMMAND = [0.0, 0.0, 0.0]
CONTROL = [1.0, 1.0, 1.0, 1.0]
OUTPUT = [0.0, 0.0, 0.0]

PORT_NAME = '/dev/ttyUSB0'

Robot_Outbond = [200, 270]
Control_Dis = 380.0
Ignore_Dis = 600.0
X_Bond = 200.0
Y_Bond = 280.0

def bond_by_angle(angle):
    bond_ang = math.degrees(math.atan(Y_Bond/X_Bond))
    if (angle < bond_ang or angle > 360-bond_ang) or (angle < 180 + bond_ang and angle > 180 - bond_ang):
        return abs(X_Bond/math.cos(math.radians(angle)))
    return abs(Y_Bond/math.sin(math.radians(angle)))

def degreeFixer(angle):
    if angle >= 360:
        return angle - 360
    if angle < 0:
        return angle + 360
    return angle

def distanceFilter(raw, angle):
    out = [1.0, 1.0]
    if raw*abs(math.sin(math.radians(angle))) < (Y_Bond+100):
        dis = raw * abs(math.cos(math.radians(angle)))
        if dis > X_Bond and dis < X_Bond + 300:
            if dis < X_Bond + 100:
                out[0] = 0.0
            else:
                out[0] = (dis - X_Bond)/200 - 0.5
    if raw*abs(math.cos(math.radians(angle))) < (X_Bond+100):
        dis = raw * abs(math.sin(math.radians(angle)))
        if dis > Y_Bond and dis < Y_Bond + 300:
            if dis < Y_Bond + 100:
                out[1] = 0.0
            else:
                out[1] = (dis - Y_Bond)/200 - 0.5
    return out

def LiDarProcess():
    lidar = RPLidar(PORT_NAME)
    try:
        print('Recording measurments... Press Crl+C to stop.')
        for scan in lidar.iter_scans():
            CONTROL = [1.0, 1.0, 1.0, 1.0]
            for point in scan:
                ang = degreeFixer(-point[1]+270.0)
                pra = distanceFilter(point[2], ang)
                if ang < 90 or ang > 270:
                    if pra[0] < CONTROL[0]:
                        CONTROL[0] = pra[0]
                if ang > 0 and ang < 180:
                    if pra[1] < CONTROL[1]:
                        CONTROL[1] = pra[1]
                if ang > 90 and ang < 270:
                    if pra[0] < CONTROL[2]:
                        CONTROL[2] = pra[0]
                if ang > 180 and ang < 360:
                    if pra[1] < CONTROL[3]:
                        CONTROL[0] = pra[1]
        print("Process result: | X_P - %f | Y_P - %f | X_N - %f | Y_N - %f |" % (CONTROL[0], CONTROL[1], CONTROL[2], CONTROL[3]))
    except KeyboardInterrupt:
        print('Stoping.')
    lidar.stop()
    lidar.disconnect()
    


def on_connect(client, userdata, flags, rc):
    print("MQTT Interface Bind Success.")
    client.subscribe("/CHASSIS/COMMAND")
    client.subscribe("/CHASSIS/AHRS/ALIG")

def on_message(client, userdata, msg):
    global CHASSIS_ANGLE
    global CONTROL
    global COMMAND
    global OUTPUT
    payload = json.loads(msg.payload.decode("utf-8"))
    if msg.topic == "/CHASSIS/AHRS/ALIG":
        CHASSIS_ANGLE = payload["Yaw"]
    elif msg.topic == "/CHASSIS/COMMAND":
        X = payload["X"]
        Y = payload["Y"]
        COMMAND[2] = payload["Phi"]
        Alpha = math.radians(360-CHASSIS_ANGLE)
        sin = math.sin(Alpha)
        cos = math.cos(Alpha)
        COMMAND[1] = X * sin + Y * cos
        COMMAND[0] = X * cos - Y * sin
        if COMMAND[0] > 0:
            OUTPUT[0] = COMMAND[0]* CONTROL[0]
        else:
            OUTPUT[0] = COMMAND[0]* CONTROL[2]
        if COMMAND[1] > 0:
            OUTPUT[1] = COMMAND[1]* CONTROL[1]
        else:
            OUTPUT[1] = COMMAND[1]* CONTROL[3]
        OUTPUT[2] = COMMAND[2]
        client.publish("/CHASSIS/SET", json.dumps({"Type": "velocity", "XSet": OUTPUT[0], "YSet": OUTPUT[1], "PhiSet": OUTPUT[2]}))

class LiDarThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)

    def run(self):
        while 1:
            LiDarProcess()



client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

print("MQTT Interface Start Binding.")

client.connect("192.168.1.2", 1883, 60)

LiDar_thread = LiDarThread()
LiDar_thread.start()

client.loop_start()

client.publish("/SYS/APP/STR", json.dumps({"Name": NAME, "Time": time.time()}))

while True:
    time.sleep(1)
    client.publish("/SYS/APP/HBT", json.dumps({"Name": NAME, "Time": time.time()}))