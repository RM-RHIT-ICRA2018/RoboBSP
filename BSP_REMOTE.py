'''
Created on Mar 8, 2018

Ver_1

@author: Miya
'''

import tkinter, threading, random
from tkinter import ttk
import paho.mqtt.client as mqtt
import json
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.animation as animation
from matplotlib import style
import BSP_ROBOT_CONFIG as ROB

style.use("ggplot")

rob = ROB.robot()

payloadM = {}
payloadP = {}
onMotor = False
onPID = False


rob.mono = 8
client = mqtt.Client()
motor_directions = (1,1,1,1)
motor_labels = []
pid_labels = [[],[],[]]
pid_entrys = [[],[],[]]
XList = range(100,0,-1)
YListS = [0] * 100
YListA = [0] * 100
YListT = [0] * 100

YAW_SET = 174
PITCH_SET = 174
CHASSIS_X = 0
CHASSIS_Y = 0
CHASSIS_PHI = 0
YAW_MOVE = 0
PITCH_MOVE = 0



init = [[],[],[]]
for i in range(3):
    for j in range(len(rob.PID_Items)):
        init[i].append(True)

warning_labels = []

PID_feedback = [[],[],[]]
PID_set = [[],[],[]]
for i in range(3):
    for j in range(len(rob.PID_Items)):
        PID_feedback[i].append(0.0)
        PID_set[i].append(0.0)
    

fig = Figure(figsize=(5,5), dpi=100)
ani_spd = fig.add_subplot(3,1,1)
ani_spd.set_title("Speed")
ani_ang = fig.add_subplot(3,1,2)
ani_ang.set_title("Angle")
ani_trq = fig.add_subplot(3,1,3)
ani_trq.set_title("Torque")



SpeedList = []
AngleList = []
TorqueList = []

UpdatingGraph = True


for i in range(rob.mono):
    SpeedList.append([0] * 100)
    AngleList.append([0] * 100)
    TorqueList.append([0] * 100)



class DataHolder(object):
#     a=1 #receive test
    input_enabled = False
    motorSpeeds = []
    motorAngles = []
    motorTorques = []
    gyro = [0.0,0.0,0.0] #[x,y,z]
    acce = [0.0,0.0,0.0] #[x,y,z]
    controlKeys = [0,0,0,0,0,0,0,0,0,0] #[w,a,s,d,q,e,up,down,left,right]
    key_press = 0
    graph_motor = 0


    def __init__(self):
        for i in range(rob.mono):
            self.motorSpeeds.append(000000)
            self.motorAngles.append(000.00)
            self.motorTorques.append(000000)
        
    def set(self, item_name, index, value):
        getattr(self, item_name)[index] = value

data = DataHolder()

        
def updateMotorToGUI(): 
    for i in range(rob.mono):
        prt = "%06d " % (data.motorSpeeds[i])
        motor_labels[i].config(text=prt)
    for i in range(rob.mono,rob.mono*2):
        prt = "%06d " % (data.motorTorques[i-rob.mono])
        motor_labels[i].config(text=prt)
    for i in range(rob.mono*2,rob.mono*3):
        prt = "%06.2f " % (data.motorAngles[i-rob.mono*2])
        motor_labels[i].config(text=prt)
    for i in range(rob.mono*3,rob.mono*3+3):
        prt = "%06d " % (data.gyro[i-rob.mono*3])
        motor_labels[i].config(text=prt)
    for i in range(rob.mono*3+3,rob.mono*3+6):
        prt = "%06d " % (data.acce[i-(rob.mono*3+3)])
        motor_labels[i].config(text=prt)
    

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    client.subscribe("/MOTOR/")
    client.subscribe("/PID_FEEDBACK/")
    
def on_message(client, userdata, msg):
    global onMotor
    global onPID
    global payloadM
    global payloadP
    if msg.topic == "/MOTOR/":
        onMotor = True
        payloadM = json.loads(msg.payload.decode())
        
    elif msg.topic == "/PID_FEEDBACK/":
        onPID = True
        payloadP = json.loads(msg.payload.decode())
        
def massageProcess():
    global onMotor
    global onPID
    global payloadM
    global payloadP
    # print("thread")
    if onMotor:
        # print("onMotor")
        motorID = payloadM.get("ID")
        speedIn = payloadM.get("Speed")
        angleIn = payloadM.get("Angle")
        torqueIn = payloadM.get("Torque")
        for i in motorID:
            data.set('motorSpeeds', i, speedIn[i])
            data.set('motorAngles', i, angleIn[i])
            data.set('motorTorques', i, torqueIn[i])
            if UpdatingGraph:
                SpeedList[i].remove(SpeedList[i][0])
                SpeedList[i].append(speedIn[i])
                AngleList[i].remove(AngleList[i][0])
                AngleList[i].append(angleIn[i])
                TorqueList[i].remove(TorqueList[i][0])
                TorqueList[i].append(torqueIn[i])

        updateMotorToGUI()
        onMotor = False
    if onPID:
        PIDs = []
        PIDs.append(payloadP.get("Ps"))
        PIDs.append(payloadP.get("Is"))
        PIDs.append(payloadP.get("Ds"))
        agree = payloadP.get("Agree")
        if agree == "True":
            warning_labels[0].config(text="All PID Settings Agree",background='#0f0')
        else:
            warning_labels[0].config(text="PID Settings Disagree",background='#f00')
        udd = True
        for i in range(3):
            PID_feedback[i] = PIDs[i]
            for j in range(len(PID_feedback[i])):
                if PID_feedback[i][j] != float(pid_labels[i][j].cget("text")):
                    if init[i][j]:
                        PID_set[i][j] = float(PID_feedback[i][j])
                        pid_text = "%04.2f" % (float(PID_feedback[i][j]))
                        pid_labels[i][j].config(text=pid_text)
                        init[i][j] = False
                    else:
                        udd = False
                        break
        if udd:
            warning_labels[1].config(text="PID Settings Updated",background='#0f0')
        else:
            warning_labels[1].config(text="PID Settings Not Updated",background='#f00')
        onPID = False








def publishPID():
    client.publish("/PID_REMOTE/", json.dumps({"Ps": PID_set[0], "Is": PID_set[1], "Ds": PID_set[2]}))


def publishControl():
    client.publish("/REMOTE/", json.dumps({"XSpeed": CHASSIS_X, "YSpeed": CHASSIS_Y, "PhiSpeed": CHASSIS_PHI, "Pitch":PITCH_MOVE, "Yaw": YAW_MOVE, "YawAngle": YAW_SET, "PitchAngle": PITCH_SET}))

    
    
def updateKeyChassisControl():
    keys = data.controlKeys
    CHASSIS_X = keys[0] - keys[2]
    CHASSIS_Y = keys[3] - keys[1]
    CHASSIS_PHI = keys[5] - keys[4]
    PITCH_MOVE = keys[6] - keys[7]
    YAW_MOVE = keys[8] - keys[9]
    print('x: ' + str(CHASSIS_X) + ' y: ' + str(CHASSIS_Y) + ' phi: ' +str(CHASSIS_PHI) + ' pitch: ' + str(PITCH_MOVE) + ' yaw: ' + str(YAW_MOVE))
    publishControl()
    
def control_key_pressed(keyNo):
    if data.input_enabled and data.controlKeys[keyNo] == 0:
        print(str(keyNo)+' Pressed')
        data.controlKeys[keyNo] = 1
        updateKeyChassisControl()
        
def control_key_released(keyNo):
    if data.input_enabled and data.controlKeys[keyNo] == 1:
        print(str(keyNo)+' Released')
        data.controlKeys[keyNo] = 0
        updateKeyChassisControl()
        
def enable_control():
    data.input_enabled = not data.input_enabled

def update_graph(i):
    if UpdatingGraph:
        ani_ang.clear()
        ani_spd.clear()
        ani_trq.clear()
        #print(""+str(SpeedList[5][99]))

        YListS = SpeedList[data.graph_motor]
        ani_spd.plot(XList,YListS)
        YListA = AngleList[data.graph_motor]
        ani_ang.plot(XList,YListA)
        YListT = TorqueList[data.graph_motor]
        ani_trq.plot(XList,YListT)

        ani_spd.set_title("Speed")
        ani_ang.set_title("Angle")
        ani_trq.set_title("Torque")

def motor_slection(obs):
    data.graph_motor = obs.get()

def clearGraph():
    SpeedList.clear()
    AngleList.clear()
    TorqueList.clear()

    for i in range(rob.mono):
        SpeedList.append([0] * 100)
        AngleList.append([0] * 100)
        TorqueList.append([0] * 100)
    print(" "+str(SpeedList[5][99]))

def freezeGraph(bott):
    global UpdatingGraph
    if UpdatingGraph:
        UpdatingGraph = False
        bott.config(text="Resume Graph")
    else:
        UpdatingGraph = True
        bott.config(text="Freeze Graph")

def updatePID():
    for i in range(3):
        for j in range(len(rob.PID_Items)):
            print(str(i)+str(j))
            pid_fig = pid_entrys[i][j].get()
            if not pid_fig == "":
                pid_text = "%04.2f" % (float(pid_fig))
                pid_labels[i][j].config(text=pid_text)
                PID_set[i][j] = float(pid_fig)
    publishPID()

def set_gimbal(yaw_entry,pitch_entry):
    global YAW_SET
    global PITCH_SET
    YAW_SET = int(yaw_entry.get())
    PITCH_SET = int(pitch_entry.get())
    publishControl()



class MsgThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)

    def run(self):
        while 1:
            massageProcess()
            


def main():
    
    #MQTT setup
    
    client.on_connect = on_connect
    client.on_message = on_message
    
    HOST = "192.168.1.2"

    print("MQTT client connecting to host ["+HOST+"]")
    
    client.connect(HOST, 1883, 60)
    
    client.loop_start()
    
    # GUI setup

    root = tkinter.Tk()

    left_frame = ttk.Frame(root, padding = 5)
    
    monitor_frame = ttk.Labelframe(left_frame, padding=10, text='Status Monitor')
    
    motors_frame = ttk.Labelframe(monitor_frame, padding=10, text='Motors')
    
    motor_num_frame = ttk.Frame(motors_frame)
    for i in range(rob.mono):  
        motor_label = ttk.Label(motor_num_frame, text='Motor'+str(i)+'    |    ')
        motor_label.grid(row=i,column=0)
        
    motor_num_frame.grid(row=0,column=0)
    
    speed_frame = ttk.Frame(motors_frame)    
    for i in range(rob.mono):   
        speed_title = ttk.Label(speed_frame, text='Speed: ')
        speed_title.grid(row=i,column=0)
        
        speed_label = ttk.Label(speed_frame, text=str(data.motorSpeeds[i])+'  ')
        speed_label.grid(row=i,column=1)
        motor_labels.append(speed_label)
        
    speed_frame.grid(row=0,column=1)
    
    torque_frame = ttk.Frame(motors_frame)    
    for i in range(rob.mono):
        torque_title = ttk.Label(torque_frame, text='Torque: ')
        torque_title.grid(row=i,column=0)
        
        torque_label = ttk.Label(torque_frame, text=str(data.motorTorques[i])+'  ')
        torque_label.grid(row=i,column=1)
        motor_labels.append(torque_label)
        
    torque_frame.grid(row=0,column=2)
    
    angle_frame = ttk.Frame(motors_frame) 
    for i in range(rob.mono):
        angle_title = ttk.Label(angle_frame, text='Angle: ')
        angle_title.grid(row=i,column=0)
        
        angle_label = ttk.Label(angle_frame, text=str(data.motorAngles[i])+'  ')
        angle_label.grid(row=i,column=1)
        motor_labels.append(angle_label)
        
    angle_frame.grid(row=0,column=3)
       
    motors_frame.grid(row=0)
    
    gyro_acce_frame = ttk.Labelframe(monitor_frame, padding=10, text='Gyro & Acce')
    
#     Gyro
    gyro_frame = ttk.Frame(gyro_acce_frame)  
    
    gyro_label = ttk.Label(gyro_frame, text='Gyro'+'    |    ')
    gyro_label.grid(row=0,column=0)
    
    gx_title = ttk.Label(gyro_frame, text='x: ')
    gx_title.grid(row=0,column=1)
    
    gx_label = ttk.Label(gyro_frame, text=str(data.gyro[0])+'  ')
    gx_label.grid(row=0,column=2)
    motor_labels.append(gx_label)
    
    gy_title = ttk.Label(gyro_frame, text='y: ')
    gy_title.grid(row=0,column=3)
    
    gy_label = ttk.Label(gyro_frame, text=str(data.gyro[1])+'  ')
    gy_label.grid(row=0,column=4)
    motor_labels.append(gy_label)
    
    gz_title = ttk.Label(gyro_frame, text='z: ')
    gz_title.grid(row=0,column=5)
    
    gz_label = ttk.Label(gyro_frame, text=str(data.gyro[2])+'  ')
    gz_label.grid(row=0,column=6)
    motor_labels.append(gz_label)
    
    gyro_frame.grid(row=0)
    
#     Acce
    acce_frame = ttk.Frame(gyro_acce_frame)  
    
    acce_label = ttk.Label(acce_frame, text='Acce'+'    |    ')
    acce_label.grid(row=0,column=0)
    
    ax_title = ttk.Label(acce_frame, text='x: ')
    ax_title.grid(row=0,column=1)
    
    ax_label = ttk.Label(acce_frame, text=str(data.acce[0])+'  ')
    ax_label.grid(row=0,column=2)
    motor_labels.append(ax_label)
    
    ay_title = ttk.Label(acce_frame, text='y: ')
    ay_title.grid(row=0,column=3)
    
    ay_label = ttk.Label(acce_frame, text=str(data.acce[1])+'  ')
    ay_label.grid(row=0,column=4)
    motor_labels.append(ay_label)
    
    az_title = ttk.Label(acce_frame, text='z: ')
    az_title.grid(row=0,column=5)
    
    az_label = ttk.Label(acce_frame, text=str(data.acce[2])+'  ')
    az_label.grid(row=0,column=6)
    motor_labels.append(az_label)
    
    acce_frame.grid(row=1)
    
    gyro_acce_frame.grid(row=1)

    
    
    monitor_frame.grid(column=0,row=0)

    control_frame = ttk.Labelframe(left_frame, padding=10, text='Control')

    enable_control_check = ttk.Checkbutton(control_frame, text='Enable Keyboard Control')
    
    enable_control_check_observer = tkinter.StringVar()
    enable_control_check['variable'] = enable_control_check_observer
    enable_control_check['command'] = lambda: enable_control()
    
    enable_control_check.grid(row=0)

    gimbal_control_frame = ttk.Labelframe(control_frame, padding=5,text='Gimbal Angle Setting')

    gimbal_entry_frame = ttk.Frame(gimbal_control_frame, padding=5)

    yaw_entry = ttk.Entry(gimbal_entry_frame,width=20)
    yaw_entry.grid(column=0,row=0)

    pitch_entry = ttk.Entry(gimbal_entry_frame,width=20)
    pitch_entry.grid(column=1,row=0)

    gimbal_entry_frame.grid()

    gimbal_set_button = ttk.Button(gimbal_control_frame, width=20, text='Set Gimbal Angle')
    gimbal_set_button['command'] = lambda: set_gimbal(yaw_entry,pitch_entry)
    gimbal_set_button.grid()

    gimbal_control_frame.grid(row=1)

    control_frame.grid(column=0,row=1)

    left_frame.grid(column=0,row=0)

    graph_frame = ttk.Labelframe(root, padding=10, text="Real-time Graphics")

    # ani_spd.plot(XList,YListS)
    # ani_ang.plot(XList,YListA)
    # ani_trq.plot(XList,YListT)

    canvas = FigureCanvasTkAgg(fig, graph_frame)
    canvas.show()
    canvas.get_tk_widget().grid()

    motor_radio_frame = ttk.Labelframe(graph_frame,padding=3,text="motor")

    motor_radios = []
    motor_radio_observer = tkinter.IntVar()
    for i in range(rob.mono):
        motor_radios.append(ttk.Radiobutton(motor_radio_frame, text=('m'+str(i)), value=i))
        motor_radios[i]['variable'] = motor_radio_observer
        motor_radios[i]['command'] = lambda: motor_slection(motor_radio_observer)
        motor_radios[i].grid(column=i, row=0)
    
    motor_radio_frame.grid()

    freeze_button = ttk.Button(graph_frame, width=20, text='Freeze Graph')
    freeze_button['command'] = (lambda: freezeGraph(freeze_button))
    freeze_button.grid()

    clear_button = ttk.Button(graph_frame, width=20, text='Clear Graph')
    clear_button['command'] = (lambda: clearGraph())
    clear_button.grid()

    graph_frame.grid(column=1,row=0)    

    PID_frame = ttk.Labelframe(root, padding=5, text="PID Adjustment")

    warning_label_0 = ttk.Label(PID_frame,text="All PID Settings Agree",background='#0f0')
    warning_label_0.grid()
    warning_labels.append(warning_label_0)

    warning_label_1 = ttk.Label(PID_frame,text="PID Settings Updated",background='#0f0')
    warning_label_1.grid()
    warning_labels.append(warning_label_1)

    p_i_d = ["P:","I:","D:"]

    for i in range(len(rob.PID_Items)):
        pid_frame = ttk.Labelframe(PID_frame,padding=1,text=rob.PID_Items[i])
        for j in range(3):
            p_i_d_frame = ttk.Labelframe(pid_frame,padding=1,text=p_i_d[j])
            p_i_d_label = ttk.Label(p_i_d_frame,text="000")
            pid_labels[j].append(p_i_d_label)
            p_i_d_label.grid()
            p_i_d_entry = ttk.Entry(p_i_d_frame)
            pid_entrys[j].append(p_i_d_entry)
            p_i_d_entry.grid()
            p_i_d_frame.grid(column=j,row=0)
        pid_frame.grid()

    clear_button = ttk.Button(PID_frame, width=20, text='Update PID Settings')
    clear_button['command'] = (lambda: updatePID())
    clear_button.grid()




    PID_frame.grid(column=3,row=0)
    
    
    
    root.bind_all('<KeyPress-w>', lambda event: control_key_pressed(0))
    root.bind_all('<KeyPress-a>', lambda event: control_key_pressed(1))
    root.bind_all('<KeyPress-s>', lambda event: control_key_pressed(2))
    root.bind_all('<KeyPress-d>', lambda event: control_key_pressed(3))
    root.bind_all('<KeyPress-q>', lambda event: control_key_pressed(4))
    root.bind_all('<KeyPress-e>', lambda event: control_key_pressed(5))
    root.bind_all('<KeyPress-Up>', lambda event: control_key_pressed(6))    
    root.bind_all('<KeyPress-Down>', lambda event: control_key_pressed(7))
    root.bind_all('<KeyPress-Left>', lambda event: control_key_pressed(8))
    root.bind_all('<KeyPress-Right>', lambda event: control_key_pressed(9))
    
    root.bind_all('<KeyRelease-w>', lambda event: control_key_released(0))
    root.bind_all('<KeyRelease-a>', lambda event: control_key_released(1))
    root.bind_all('<KeyRelease-s>', lambda event: control_key_released(2))
    root.bind_all('<KeyRelease-d>', lambda event: control_key_released(3))
    root.bind_all('<KeyRelease-q>', lambda event: control_key_released(4))
    root.bind_all('<KeyRelease-e>', lambda event: control_key_released(5))
    root.bind_all('<KeyRelease-Up>', lambda event: control_key_released(6))    
    root.bind_all('<KeyRelease-Down>', lambda event: control_key_released(7))
    root.bind_all('<KeyRelease-Left>', lambda event: control_key_released(8))
    root.bind_all('<KeyRelease-Right>', lambda event: control_key_released(9))
    
    ani = animation.FuncAnimation(fig, update_graph, interval=100)
    
    Msg_thread = MsgThread()
    Msg_thread.start()

    root.mainloop()
    
    
    
        

main()   
       
    
    
    

# if __name__ == '__main__':
#     main()
    
