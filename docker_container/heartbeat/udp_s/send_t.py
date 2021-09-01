import socket
import time
import json

HOST = '127.0.0.1'
PORT = 4000

SOCK = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

ID = 1
while 1:
    try:
        SOCK = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        SOCK.connect((HOST, PORT))
        #now = rospy.Time.now()
        msg_s = {"msg": ID, "time": time.time()}

        data = json.dumps(msg_s, default=str)

        time.sleep(1)
        print ("sending data %s" % data)
        SOCK.sendall(data.encode('utf-8'))

        ID = ID + 1
        if ID == 1000:
            break
        SOCK.close()
    except Exception as e:
        print(e)
