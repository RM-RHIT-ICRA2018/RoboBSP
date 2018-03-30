import serial,time,re, json
import madgwick_py.madgwickahrs as _ahrs
from datetime import datetime
import paho.mqtt.client as mqtt
ahrs = _ahrs.MadgwickAHRS(0.00677, 1.0)

r = re.compile(r'''
    \s*                # Any whitespace.
    (                  # Start capturing here.
      [^,"']+?         # Either a series of non-comma non-quote characters.
      |                # OR
      "(?:             # A double-quote followed by a string of characters...
          [^"\\]|\\.   # That are either non-quotes or escaped...
       )*              # ...repeated any number of times.
      "                # Followed by a closing double-quote.
      |                # OR
      '(?:[^'\\]|\\.)*'# Same as above, for single quotes.
    )                  # Done capturing.
    \s*                # Allow arbitrary space before the comma.
    (?:,|$)            # Followed by a comma or the end of a string.
    ''', re.VERBOSE)

ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)

ser.readline()
a = 0
connect_status = False

def on_connect(client, userdata, flags, rc):
    global connect_status
    print("Connected with result code "+str(rc))
    connect_status = True

client = mqtt.Client()

client.on_connect = on_connect
client.connect("192.168.1.2", 1883, 60)

client.loop_start()

while 1:
    try:
        imuIn = (ser.readline()).decode("utf-8")
    except:
        continue
    imu_raw = r.findall(imuIn)
    imu = []
    for k in range(len(imu_raw)):
        if k in range(1,4):
            imu.append(float(imu_raw[k])*(0.0174532925 ))
        else:
            imu.append(float(imu_raw[k])/1000)

    ahrs.update(imu[1:4] , imu[4:7], imu[7:10])
    pitch, roll, yaw = (ahrs.quaternion).to_euler_angles()
    if connect_status == True:
        client.publish("/IMU/RAW", json.dumps({"GyroX": imu_raw[1],"GyroY": imu_raw[2],"GyroZ":imu_raw[3],"AccX":imu_raw[4],"AccY":imu_raw[5], "AccZ":imu_raw[6], "TempX":imu_raw[7], "TempY":imu_raw[8], "TempZ":imu_raw[9]}))
        client.publish("/IMU/AHRS", json.dumps({"Roll": roll/(0.0174532925), "Pitch": pitch/(0.0174532925), "Yaw": yaw/(0.0174532925), "GyroRoll":float(imu_raw[1]), "GyroPitch":float(imu_raw[2]), "GyroYaw":float(imu_raw[3])}))
    if a > 10:
        print("\rROLL: %.06f, PITCH: %.06f, YAW: %.06f" % (roll/(0.0174532925 ), pitch/(0.0174532925 ), yaw/(0.0174532925 )), end="")
        a = 0
    else:
        a = a+1
