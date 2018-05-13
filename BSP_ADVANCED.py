import json, time, threading, math
import BSP_ERROR, BSP_PID as PID
import BSP_ROBOT_CONFIG as ROB
import copy
import paho.mqtt.client as mqtt

rob = ROB.robot()
rob.mono = 7

Remote_Yaw = 0
Remote_Pitch = 0

CHASSIS_TYPE = "none"

PID_SETTINGS_SET = []  
PID_SETTINGS_SET.append({"P":0.0, "I":0.0, "D":0.0})        #Chassis_X
PID_SETTINGS_SET.append({"P":0.0, "I":0.0, "D":0.0})            #Chassis_Y
PID_SETTINGS_SET.append({"P":0.0, "I":0.0, "D":0.0})          #Chassis_Phi

PID_SETTINGS_SET.append({"P":0.0, "I":0.0, "D":0.0})            #Yaw_Image
PID_SETTINGS_SET.append({"P":0.0, "I":0.0, "D":0.0})          #Pitch_image


PID_NUM = len(rob.PID_Items_ADVANCED)

PID_SETTINGS_REAL = copy.deepcopy(PID_SETTINGS_SET)

PIDs = []

CONFIG_TYPE = [] #[c1,c2,c3,c4,yaw,pitch,feed]
CONFIG_SET = [] #[c1,c2,c3,c4,yaw,pitch,feed]

DATA_COLLECT = []

adv_updated_origin = [True, False]
adv_updated_real = copy.deepcopy(adv_updated_origin)

for i in range(rob.mono):
    CONFIG_TYPE.append("None")
    CONFIG_SET.append(0.0)
    DATA_COLLECT.append(0.0)


PID_SetPoints = [0.0, 0.0, 0.0, 0.0, 0.0]

for i in range(PID_NUM):
    PIDs.append(PID.PID(PID_SETTINGS_REAL[i]["P"], PID_SETTINGS_REAL[i]["I"], PID_SETTINGS_REAL[i]["D"]))
    PIDs[i].SetPoint=PID_SetPoints[i]
    PIDs[i].setSampleTime(0.0001)

def update_PID():
    for i in range(PID_NUM):
        PIDs[i].clear()
        PIDs[i].setKp(PID_SETTINGS_REAL[i]["P"])
        PIDs[i].setKi(PID_SETTINGS_REAL[i]["I"])
        PIDs[i].setKd(PID_SETTINGS_REAL[i]["D"])

def publish_config():
    client.publish("/CONFIG/", json.dumps({"Type": CONFIG_TYPE, "Set": CONFIG_SET, "PassData": DATA_COLLECT}))

def on_connect(client, userdata, flags, rc):
    print(BSP_ERROR.notice("MQTT Interface Bind Success."))
    client.subscribe("/CHASSIS/SET")
    client.subscribe("/CHASSIS_STATUS/#")
    client.subscribe("/GIMBAL/SET")
    # client.subscribe("/MOTOR/#")
    client.subscribe("/REMOTE/#")
    client.subscribe("/PID_REMOTE/#")
    client.subscribe("/CHASSIS/#")
    client.subscribe("/UWB/PUS")
    client.subscribe("/")

def on_message(client, userdata, msg):
    global CHASSIS_TYPE
    if msg.topic != "/MOTOR/":
        print(BSP_ERROR.info((" Time: %08.5f" % time.time()) + "Topic: "+ msg.topic + " Payload: " + msg.payload.decode("utf-8")))
    payload = json.loads(msg.payload.decode("utf-8"))
    if msg.topic == "/REMOTE/":
        PIDs[0].SetPoint = payload["XSpeed"]
        PIDs[1].SetPoint = payload["YSpeed"]
        PIDs[2].SetPoint = payload["PhiSpeed"]

    elif msg.topic == "/CHASSIS/SET":
        CHASSIS_TYPE = payload["Type"]
        PIDs[0].SetPoint = float(payload["XSet"])
        PIDs[1].SetPoint = float(payload["YSet"])
        PIDs[2].SetPoint = float(payload["PhiSet"])

    elif msg.topic == "/GIMBAL/SET":
        if payload["Type"] == "None":
            # pass
            for i in range(4,6):
                CONFIG_TYPE[i] = "None"
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
            # print(str(payload["dX"]))
            for i in range(4,6):
                CONFIG_TYPE[i] = "Lower"
            PIDs[3].update(payload["dX"])
            PIDs[4].update(payload["dY"])
            # print(str(PIDs[3].output))
            DATA_COLLECT[4] = payload["dX"]
            DATA_COLLECT[5] = payload["dY"]
            CONFIG_SET[4] = PIDs[3].output
            CONFIG_SET[5] = PIDs[4].output
            # adv_updated_real[1] = True


    elif msg.topic == "/CHASSIS_STATUS/VELOCITY":
        if CHASSIS_TYPE == "velocity":
            PIDs[0].update(payload["X"])
            PIDs[1].update(payload["Y"])
            chassis_output()

    elif msg.topic == "/UWB/PUS":
        if CHASSIS_TYPE == "position":
            PIDs[0].update(payload["posX"])
            PIDs[1].update(payload["posY"])
            chassis_output()

    elif msg.topic == "/CHASSIS/AHRS":
        if CHASSIS_TYPE == "position":
            PIDs[2].update(payload["Yaw"])
            chassis_output()

    elif msg.topic == "/CHASSIS/RAW":
        if CHASSIS_TYPE == "velocity":
            PIDs[2].update(payload["GyroZ"])
            chassis_output()

    elif msg.topic == "/PID_REMOTE/":
        Ps = payload.get("Ps")
        Is = payload.get("Is")
        Ds = payload.get("Ds")
        jump = len(rob.PID_Items_BASIC)
        for i in range(PID_NUM):
            # print(str(Ps[jump + i]))
            PID_SETTINGS_REAL[i]["P"] = Ps[jump + i]
            PID_SETTINGS_REAL[i]["I"] = Is[jump + i]
            PID_SETTINGS_REAL[i]["D"] = Ds[jump + i]
        # print(str(PID_SETTINGS_REAL[0]["P"]))
        update_PID()
        publish_real_pid()
        for i in range(PID_NUM):
            print(str(PIDs[i].output))
    
    publish_config()

def compare_pid():
    for i in range(int(PID_NUM)):
        if PID_SETTINGS_REAL[i]["P"] != PID_SETTINGS_SET[i]["P"] or PID_SETTINGS_REAL[i]["I"] != PID_SETTINGS_SET[i]["I"] or PID_SETTINGS_REAL[i]["D"] != PID_SETTINGS_SET[i]["D"]:
            return "False"
    return "True"

def publish_real_pid():
    #pdb.set_trace()
    Ps = []
    Is = []
    Ds = []
    for i in range(int(PID_NUM)):
        Ps.append(PIDs[i].getP())
        Is.append(PIDs[i].getI())
        Ds.append(PIDs[i].getD())
    agree = compare_pid()
    pid_msg = {"Ps":Ps, "Is":Is, "Ds":Ds, "Agree": agree}
    client.publish("/PID_FEEDBACK/ADVANCED", json.dumps(pid_msg))

def chassis_output():
    X_OUT = PIDs[0].output
    Y_OUT = PIDs[1].output
    Phi_OUT = PIDs[2].output
    Chassis_OUT = [-X_OUT + Y_OUT + Phi_OUT, X_OUT + Y_OUT + Phi_OUT, X_OUT - Y_OUT + Phi_OUT, -X_OUT - Y_OUT + Phi_OUT]
    for i in range(4):
        CONFIG_TYPE[i] = "UPPER"
        CONFIG_SET[i] = Chassis_OUT[i]

# def publishAdv():
#     global adv_updated_real
#     global adv_updated_origin
#     if False not in adv_updated_real:
#         client.publish("/ADVANCE/", json.dumps({"Data": DATA_COLLECT, "Advance": CONFIG_SET}))
#         adv_updated_real = copy.deepcopy(adv_updated_origin)


# class PubThread(threading.Thread):
#     def __init__(self):
#         threading.Thread.__init__(self)

#     def run(self):
#         while 1:
#             publishAdv()



client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

print(BSP_ERROR.info("MQTT Interface Start Binding."))

client.connect("127.0.0.1", 1883, 60)

# Pub_thread = PubThread()
# Pub_thread.start()

client.loop_forever()
        
        

