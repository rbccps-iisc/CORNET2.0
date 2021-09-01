import time
import socket
import sys

# import rospy
import traceback

from threading import Thread


def client_thread(conn):
    is_active = True
    data = conn.recv(1024)

    print(data)

    is_active = False
    conn.close()


def start_server():
    # rospy.init_node("receiver")  # initial ros node

    HOST = ''  # Standard loopback interface address (localhost)
    PORT = 4000  # Port to listen on (non-privileged ports are > 1023)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    # Bind socket to local host and port
    try:
        s.bind((HOST, PORT))
    except socket.error as msg:
        print ("Bind failed. Error Code : %s message % " % (str(msg[0]), str(msg[1])))
        sys.exit()

    s.listen(1000)

    while True:
        conn, addr = s.accept()

        try:
            Thread(target=client_thread, args=[conn]).start()
        except:
            print("Thread did not start.")
            traceback.print_exc()
    s.close()


def main():
    start_server()


if __name__ == "__main__":
    main()
