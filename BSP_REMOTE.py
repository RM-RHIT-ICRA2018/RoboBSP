'''
Created on Mar 8, 2018

Ver_1

@author: Miya
'''

import tkinter, threading, random
from tkinter import ttk
import paho.mqtt.client as mqtt
import json

motor_num = 7
client = mqtt.Client()
motor_directions = (1,1,1,1)
labels = []

class DataHolder(object):
#     a=1 #receive test
    input_enabled = False
    motorSpeeds = []
    motorPositions = []
    motorTorques = []
    gyro = [0.0,0.0,0.0] #[x,y,z]
    acce = [0.0,0.0,0.0] #[x,y,z]
    controlKeys = [0,0,0,0,0,0] #[w,a,s,d,q,e]


    def __init__(self):
        for i in range(motor_num):
            self.motorSpeeds.append(0)
            self.motorPositions.append(0.0)
            self.motorTorques.append(0)
        
    def set(self, item_name, index, value):
        getattr(self, item_name)[index] = value

data = DataHolder()

        
def updateToGUI(): 
    for i in range(motor_num):
        labels[i].config(text=str(data.motorSpeeds[i])+'  ')
    for i in range(motor_num,motor_num*2):
        labels[i].config(text=str(data.motorTorques[i-motor_num])+'  ')
    for i in range(motor_num*2,motor_num*3):
        labels[i].config(text=str(data.motorPositions[i-motor_num*2])+'  ')
    for i in range(motor_num*3,motor_num*3+3):
        labels[i].config(text=str(data.gyro[i-motor_num*3])+'  ')
    for i in range(motor_num*3+3,motor_num*3+6):
        labels[i].config(text=str(data.acce[i-(motor_num*3+3)])+'  ')

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    client.subscribe("/MOTOR/")
    
def on_message(client, userdata, msg):
    payload = json.loads(msg.payload.decode())
    motorID = payload.get("ID")
    speedIn = payload.get("Speed")
    positionIn = payload.get("Angle")
    torqueIn = payload.get("Torque")
    data.set('motorSpeeds', motorID, speedIn)
    data.set('motorPositions', motorID, positionIn)
    data.set('motorTorques', motorID, torqueIn)
    updateToGUI()
    
    
def sentControlMsg():
    keys = data.controlKeys
    x = keys[0] - keys[2]
    y = keys[3] - keys[1]
    phi = keys[5] - keys[4]
    print('x: ' + str(x) + ' y: ' + str(y) + ' phi: ' +str(phi) )
    client.publish("/REMOTE/", json.dumps({"XSpeed": x, "YSpeed": y, "PhiSpeed": phi}))
    
def control_key_pressed(keyNo):
#     #receive test
#     a=data.a 
#     client.publish("/MOTOR/", json.dumps({"ID": 1, "Speed": a, "Angle": a/(a-0.5), "Torque": a*a}))
#     client.publish("/MOTOR/", json.dumps({"ID": 3, "Speed": 2*a, "Angle": a-a*0.02+a/3, "Torque": a*a-a}))
#     data.a = data.a+1
    
    if data.input_enabled:
        data.controlKeys[keyNo] = (data.controlKeys[keyNo] + 1)%2
        sentControlMsg()
        
def enable_control():
    data.input_enabled = not data.input_enabled
    

def main():
    
#    MQTT setup
    
    client.on_connect = on_connect
    client.on_message = on_message
    
    HOST = "mosquitto.csse.rose-hulman.edu"
    
    client.connect(HOST, 1883, 60)
    
    client.loop_start()
    
#    GUI setup
    
    root = tkinter.Tk()
    
    monitor_frame = ttk.Labelframe(root, padding=10, text='Status Monitor')
    
    motors_frame = ttk.Labelframe(monitor_frame, padding=10, text='Motors')
    
    motor_num_frame = ttk.Frame(motors_frame)
    for i in range(motor_num):  
        motor_label = ttk.Label(motor_num_frame, text='Motor'+str(i)+'    |    ')
        motor_label.grid(row=i,column=0)
        
    motor_num_frame.grid(row=0,column=0)
    
    speed_frame = ttk.Frame(motors_frame)    
    for i in range(motor_num):   
        speed_title = ttk.Label(speed_frame, text='Speed: ')
        speed_title.grid(row=i,column=0)
        
        speed_label = ttk.Label(speed_frame, text=str(data.motorSpeeds[i])+'  ')
        speed_label.grid(row=i,column=1)
        labels.append(speed_label)
        
    speed_frame.grid(row=0,column=1)
    
    torque_frame = ttk.Frame(motors_frame)    
    for i in range(motor_num):
        torque_title = ttk.Label(torque_frame, text='Torque: ')
        torque_title.grid(row=i,column=0)
        
        torque_label = ttk.Label(torque_frame, text=str(data.motorTorques[i])+'  ')
        torque_label.grid(row=i,column=1)
        labels.append(torque_label)
        
    torque_frame.grid(row=0,column=2)
    
    position_frame = ttk.Frame(motors_frame) 
    for i in range(motor_num):
        position_title = ttk.Label(position_frame, text='Position: ')
        position_title.grid(row=i,column=0)
        
        position_label = ttk.Label(position_frame, text=str(data.motorPositions[i])+'  ')
        position_label.grid(row=i,column=1)
        labels.append(position_label)
        
    position_frame.grid(row=0,column=3)
       
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
    labels.append(gx_label)
    
    gy_title = ttk.Label(gyro_frame, text='y: ')
    gy_title.grid(row=0,column=3)
    
    gy_label = ttk.Label(gyro_frame, text=str(data.gyro[1])+'  ')
    gy_label.grid(row=0,column=4)
    labels.append(gy_label)
    
    gz_title = ttk.Label(gyro_frame, text='z: ')
    gz_title.grid(row=0,column=5)
    
    gz_label = ttk.Label(gyro_frame, text=str(data.gyro[2])+'  ')
    gz_label.grid(row=0,column=6)
    labels.append(gz_label)
    
    gyro_frame.grid(row=0)
    
#     Acce
    acce_frame = ttk.Frame(gyro_acce_frame)  
    
    acce_label = ttk.Label(acce_frame, text='Acce'+'    |    ')
    acce_label.grid(row=0,column=0)
    
    ax_title = ttk.Label(acce_frame, text='x: ')
    ax_title.grid(row=0,column=1)
    
    ax_label = ttk.Label(acce_frame, text=str(data.acce[0])+'  ')
    ax_label.grid(row=0,column=2)
    labels.append(ax_label)
    
    ay_title = ttk.Label(acce_frame, text='y: ')
    ay_title.grid(row=0,column=3)
    
    ay_label = ttk.Label(acce_frame, text=str(data.acce[1])+'  ')
    ay_label.grid(row=0,column=4)
    labels.append(ay_label)
    
    az_title = ttk.Label(acce_frame, text='z: ')
    az_title.grid(row=0,column=5)
    
    az_label = ttk.Label(acce_frame, text=str(data.acce[2])+'  ')
    az_label.grid(row=0,column=6)
    labels.append(az_label)
    
    acce_frame.grid(row=1)
    
    gyro_acce_frame.grid(row=1)
    
    update_button = ttk.Button(monitor_frame, width=20, text='Update')
    update_button['command'] = (lambda: updateToGUI())
    update_button.grid(row=2)

    enable_control_check = ttk.Checkbutton(monitor_frame, text='Enable Keyboard Control')
    
    enable_control_check_observer = tkinter.StringVar()
    enable_control_check['variable'] = enable_control_check_observer
    enable_control_check['command'] = lambda: enable_control()
    
    enable_control_check.grid(row=3)
    
    monitor_frame.grid(column=0)
    
    
    
    root.bind_all('<Key-w>', lambda event: control_key_pressed(0))
    root.bind_all('<Key-a>', lambda event: control_key_pressed(1))
    root.bind_all('<Key-s>', lambda event: control_key_pressed(2))
    root.bind_all('<Key-d>', lambda event: control_key_pressed(3))
    root.bind_all('<Key-q>', lambda event: control_key_pressed(4))
    root.bind_all('<Key-e>', lambda event: control_key_pressed(5))
    
    GUI_thread = threading.Thread(target=root.mainloop())
    GUI_thread.start() 
    
        
    
    
       
    
    
    

if __name__ == '__main__':
    main()
    
