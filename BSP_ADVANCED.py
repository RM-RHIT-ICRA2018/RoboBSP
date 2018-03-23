import json, time, threading, math
import BSP_ERROR, BSP_PID as PID
import BSP_ROBOT_CONFIG as ROB
import copy
import paho.mqtt.client as mqtt

rob = ROB.robot()
rob.mono = 7

Remote_Yaw = 0
Remote_Pitch = 0

PID_SETTINGS_SET = []  
PID_SETTINGS_SET.append({"P":0.0, "I":0.0, "D":0.0})        #Chassis_X
PID_SETTINGS_SET.append({"P":0.0, "I":0.0, "D":0.0})            #Chassis_Y
PID_SETTINGS_SET.append({"P":0.0, "I":0.0, "D":0.0})          #Chassis_Phi

PID_SETTINGS_SET.append({"P":0.0, "I":0.0, "D":0.0})            #Yaw
PID_SETTINGS_SET.append({"P":0.0, "I":0.0, "D":0.0})          #Pitch

PID_NUM = len(PID_SETTINGS_SET)

PID_SETTINGS_REAL = copy.deepcopy(PID_SETTINGS_SET)

PIDs = []

CONFIG_TYPE = [] #[c1,c2,c3,c4,yaw,pitch,feed]
CONFIG_SET = [] #[c1,c2,c3,c4,yaw,pitch,feed]

for i in range(rob.mono):
    CONFIG_TYPE.append("Upper")
    CONFIG_SET.append(0.0)


PID_SetPoints = [0.0, 0.0, 0.0, 0.0, 0.0]

for i in range(PID_NUM):
    PIDs.append(PID.PID(PID_SETTINGS_REAL[i]["P"], PID_SETTINGS_REAL[i]["I"], PID_SETTINGS_REAL[i]["D"]))
    PIDs[i].SetPoint=PID_SetPoints[i]
    PIDs[i].setSampleTime(0.01)

def update_PID():
    for i in range(PID_NUM):
        PIDs[i].clear()
        PIDs[i].setKp(PID_SETTINGS_REAL[i]["P"])
        PIDs[i].setKi(PID_SETTINGS_REAL[i]["I"])
        PIDs[i].setKd(PID_SETTINGS_REAL[i]["D"])

def publish_config():
    client.publish("/CONFIG/", json.dumps({"Type": CONFIG_TYPE, "Set": CONFIG_SET))

def on_connect(client, userdata, flags, rc):
    print(BSP_ERROR.notice("MQTT Interface Bind Success."))
    client.subscribe("/CHASSIS_SET/#")
    client.subscribe("/CHASSIS_STATUS/#")
    client.subscribe("/GIMBAL_SET/#")
    client.subscribe("/MOTOR/#")
    client.subscribe("/REMOTE/#")
    client.subscribe("/")

def on_message(client, userdata, msg):
    print(BSP_ERROR.info("Topic: "+ msg.topic + " Payload: " + msg.payload.decode("utf-8")))
    payload = json.loads(msg.payload.decode("utf-8"))
    if msg.topic == "/REMOTE/":
        PIDs[0].SetPoint = payload["XSpeed"]
        PIDs[1].SetPoint = payload["YSpeed"]
        PIDs[2].SetPoint = payload["PhiSpeed"]

    elif msg.topic == "/CHASSIS_SET/":
        PIDs[0].SetPoint = payload["XSet"]
        PIDs[1].SetPoint = payload["YSet"]
        PIDs[2].SetPoint = payload["PhiSet"]

    elif msg.topic == "/GIMBAL_SET/":
        if payload["Type"] == "Angle":
            for i in range(4,6):
                CONFIG_TYPE[i] = "Upper"
            CONFIG_SET[4] = payload["YawSet"]
            CONFIG_SET[5] = payload["PitchSet"]
        elif payload["Type"] == "Speed":
            for i in range(4,6):
                CONFIG_TYPE[i] = "Lower"  
            CONFIG_SET[4] = payload["YawSet"]
            CONFIG_SET[5] = payload["PitchSet"]      
        elif payload["Type"] == "Image":
            for i in range(4,6):
                CONFIG_TYPE[i] = "Lower"
            PIDs[3].update(payload["Yaw"])
            PIDs[4].update(payload["Pitch"])
            CONFIG_SET[4] = PIDs[3].output
            CONFIG_SET[5] = PIDs[4].output

    elif msg.topic == "/CHASSIS_STATUS/":
        PIDs[0].update(payload["XSpeed"])
        PIDs[1].update(payload["YSpeed"])
        PIDs[2].update(payload["PhiSpeed"])
        chassis_output()
    
    publish_config()

def chassis_output():
    X_OUT = PIDs[0].output
    Y_OUT = PIDs[1].output
    Phi_OUT = PIDs[2].output
    Chassis_OUT = [-X_OUT + Y_OUT + Phi_OUT, X_OUT + Y_OUT + Phi_OUT, X_OUT - Y_OUT + Phi_OUT, -X_OUT - Y_OUT + Phi_OUT]
    for i in range(4):
        CONFIG_SET[i] = Chassis_OUT[i]



client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

print(BSP_ERROR.info("MQTT Interface Start Binding."))

client.connect("127.0.0.1", 1883, 60)
client.loop_forever()
        
        

