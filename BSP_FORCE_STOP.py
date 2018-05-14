import json
import paho.mqtt.client as mqtt

def on_connect(client, userdata, flags, rc):
    print(BSP_ERROR.notice("MQTT Interface Bind Success."))

def on_message(client, userdata, msg):
    pass

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

print(BSP_ERROR.info("MQTT Interface Start Binding."))

client.connect("127.0.0.1", 1883, 60)

client.loop_start()

while True:
    client.publish("/FAILSAFE/", json.dumps("Type": "ForceShutDown"))