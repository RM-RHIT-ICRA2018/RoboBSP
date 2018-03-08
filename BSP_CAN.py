import socket, struct, sys, json, time
fmt = "<IB3x8s"

sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
interface = "can0"
try:
   sock.bind((interface,))
except OSError:
   sys.stderr.write("Could not bind to interface '%s'\n" % interface)
   exit()
a = []
while 1:
   can_pkt = sock.recv(16)
   can_id, length, data = struct.unpack(fmt, can_pkt)
   can_id &= socket.CAN_EFF_MASK
   data = data[:length]

   if can_id >= 0x200 and can_id <= 0x208:
       torque = data[4]*256+data[5]
       if torque >= 2**15:
           torque = -(2**16-torque)
       speed = data[2]*256+data[3]
       if speed >= 2**15:
           speed = -(2**16-speed)
       msg_content = {"Angle" : (360.0)/(8191)*(data[0]*256+data[1]),
"Speed" : speed, "Torque" : torque, "CAN ID" : can_id}
       print(msg_content)

   can_id = 0x200
   can_pkt = struct.pack(fmt, 0x200,
8,bytes([0x0F,0xA0,0x00,0xA0,0x00,0xA0,0x00,0xA0]))
#    sock.send(can_pkt)
