import json, time, threading, math
import BSP_ERROR, BSP_PID as PID
import BSP_ROBOT_CONFIG as ROB
import copy
import paho.mqtt.client as mqtt

NAME = "BSP_ADVANCED"

rob = ROB.robot()
rob.mono = 7

Remote_Yaw = 0
Remote_Pitch = 0

CHASSIS_READY = [False, False, False]

CHASSIS_TYPE = "position"

CHASSIS_ANGLE = 0

Target = False

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
    PIDs[i].setWindup(20)

PIDs[2].setDegreeFixer(True)

def update_PID():
    for i in range(PID_NUM):
        PIDs[i].clear()
        PIDs[i].setKp(PID_SETTINGS_REAL[i]["P"])
        PIDs[i].setKi(PID_SETTINGS_REAL[i]["I"])
        PIDs[i].setKd(PID_SETTINGS_REAL[i]["D"])

def publish_config():
    global CONFIG_SET
    global CONFIG_TYPE
    #for i in range(4):
    #    CONFIG_TYPE[i] = "Upper"
    #    CONFIG_SET[i] = 100
    client.publish("/CONFIG/", json.dumps({"Type": CONFIG_TYPE, "Set": CONFIG_SET, "PassData": DATA_COLLECT}))

def on_connect(client, userdata, flags, rc):
    print(BSP_ERROR.notice("MQTT Interface Bind Success."))
    client.subscribe("/CHASSIS/SET")
    client.subscribe("/CHASSIS_STATUS/#")
    client.subscribe("/GIMBAL/SET")
    client.subscribe("/MOTOR/#")
    client.subscribe("/REMOTE/#")
    client.subscribe("/PID_REMOTE/#")
    client.subscribe("/CHASSIS/#")
    client.subscribe("/UWB/POS")
    client.subscribe("/")

def on_message(client, userdata, msg):
    global CHASSIS_TYPE
    global CHASSIS_ANGLE
    global Target
    # if msg.topic != "/MOTOR/":
        # print(BSP_ERROR.info((" Time: %08.5f" % time.time()) + "Topic: "+ msg.topic + " Payload: " + msg.payload.decode("utf-8")))
    # print(str(msg.payload) + " "+ msg.topic)

    payload = json.loads(msg.payload.decode("utf-8"))
    if msg.topic == "/REMOTE/":
        PIDs[0].SetPoint = payload["XSpeed"]
        PIDs[1].SetPoint = payload["YSpeed"]
        PIDs[2].SetPoint = payload["PhiSpeed"]

    elif msg.topic == "/CHASSIS/SET":
        print("%f  %f  %f" % (payload["XSet"], payload["YSet"], payload["PhiSet"]))
        CHASSIS_TYPE = payload["Type"]
        PIDs[0].SetPoint = float(payload["XSet"])
        PIDs[1].SetPoint = float(payload["YSet"])
        PIDs[2].SetPoint = float(payload["PhiSet"])
        if payload["Type"] == "velocity":
            chassis_output()

    elif msg.topic == "/GIMBAL/SET":
        if payload["Type"] == "None":
            # pass
            for i in range(4,6):
                CONFIG_TYPE[i] = "None"
        if payload["Type"] == "Angle":
            if not Target:
                for i in range(4,6):
                    CONFIG_TYPE[i] = "Upper"
                CONFIG_SET[4] = payload["YawSet"]
                CONFIG_SET[5] = payload["PitchSet"]
        elif payload["Type"] == "Speed":
            if not Target:
                for i in range(4,6):
                    CONFIG_TYPE[i] = "Lower"
                CONFIG_SET[4] = payload["YawSet"]
                CONFIG_SET[5] = payload["PitchSet"]
        elif payload["Type"] == "Image":
            if payload["Target"] == "Positive":
                Target = True
                for i in range(4,6):
                    CONFIG_TYPE[i] = "DaUpper"
                    ddx = math.atan(payload["dX"]/1500)*90/math.pi
                    ddy = math.atan(payload["dY"]/1500)*90/math.pi
                PIDs[3].update(ddx)
                PIDs[4].update(ddy)
                # print(str(PIDs[3].output))
                DATA_COLLECT[4] = ddx
                DATA_COLLECT[5] = ddy
                CONFIG_SET[4] = PIDs[3].output
                CONFIG_SET[5] = PIDs[4].output
                # adv_updated_real[1] = True
            else:
                Target = False
                for i in range(4,6):
                    CONFIG_TYPE[i] = "Upper"
                CONFIG_SET[4] = 174
                CONFIG_SET[5] = 160


    elif msg.topic == "/CHASSIS_STATUS/VELOCITY":
        if CHASSIS_TYPE == "velocity":
            PIDs[0].update(payload["X"])
            PIDs[1].update(payload["Y"])
            chassis_output()

    elif msg.topic == "/UWB/POS":
        if CHASSIS_TYPE == "position":
            PIDs[0].update(payload["posX"])
            if not CHASSIS_READY[0]:
                PIDs[0].SetPoint = payload["posX"]
                CHASSIS_READY[0] = True
            PIDs[1].update(payload["posY"])
            if not CHASSIS_READY[1]:
                PIDs[1].SetPoint = payload["posY"]
                CHASSIS_READY[1] = True
            chassis_output()

    elif msg.topic == "/CHASSIS/AHRS/ALIG":
        CHASSIS_ANGLE = payload["Yaw"]
        if CHASSIS_TYPE == "position":
            PIDs[2].update(payload["Yaw"])
            if not CHASSIS_READY[2]:
                PIDs[2].SetPoint = payload["Yaw"]
                CHASSIS_READY[2] = True
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

def chassis_decode(X, Y, Phi, Angle):
    Alpha = math.radians(360-Angle)
    sin = math.sin(Alpha)
    cos = math.cos(Alpha)
    rY = X * sin + Y * cos
    rX = X * cos - Y * sin
    print("X: %f, Y: %f, Phi: %f, Angle: %f, rX: %f, rY: %f" % (X,Y,Phi,Angle,rX,rY))
    return [rX-rY+Phi, rX+rY+Phi, -rX+rY+Phi, -rX-rY+Phi]

def chassis_output():
    global CHASSIS_ANGLE
    global CHASSIS_TYPE
    if CHASSIS_TYPE == "position":
        if CHASSIS_READY[0] and CHASSIS_READY[1] and CHASSIS_READY[2]:
            X_OUT = PIDs[0].output
            Y_OUT = PIDs[1].output
            Phi_OUT = PIDs[2].output
            Chassis_OUT = chassis_decode(X_OUT, Y_OUT, Phi_OUT, CHASSIS_ANGLE)
            for i in range(4):
                CONFIG_TYPE[i] = "Upper"
                CONFIG_SET[i] = (float)(Chassis_OUT[i])
    elif CHASSIS_TYPE == "velocity":
        rX = PIDs[0].SetPoint
        rY = PIDs[1].SetPoint
        Phi = PIDs[2].SetPoint
        Chassis_OUT = [rX-rY+Phi, rX+rY+Phi, -rX+rY+Phi, -rX-rY+Phi]
        for i in range(4):
            CONFIG_TYPE[i] = "Upper"
            CONFIG_SET[i] = (float)(Chassis_OUT[i])


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

client.loop_start()

client.publish("/SYS/APP/STR", json.dumps({"Name": NAME, "Time": time.time()}))

while True:
    time.sleep(1)
    client.publish("/SYS/APP/HBT", json.dumps({"Name": NAME, "Time": time.time()}))



