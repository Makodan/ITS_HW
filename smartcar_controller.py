#!/usr/bin/env python

import socket
import time
from picamera import PiCamera
import RPi.GPIO as GPIO
import sys
from threading import Thread, Lock


UDP_IP = '192.168.0.111'  # The remote host
UDP_PORT = 50005  # The same port as used by the server
# HOST = sys.argv[1]
# PORT = sys.argv[2]
IsActive = True

def main():
    global IsActive
    global sendActive
    try:
        Thread(target=camera_stream, args=(), name="CameraStream").start()
        # while True:
        #     pass

    except KeyboardInterrupt:
        IsActive = False
        sendActive = False

    except:
        print("Thread didn't start. Error : " + str(sys.exc_info()))


def camera_stream():
    global IsActive

    try:
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        client_socket.connect((UDP_IP, UDP_PORT))
        connection = client_socket.makefile('wb')

        camera = PiCamera()
        camera.resolution = (640, 480)
        camera.framerate = 24
        camera.start_recording(connection, format='mjpeg')
        print("[Info] Stream started")
        while IsActive:
            camera.wait_recording()

        camera.stop_recording()

    except Exception as e:
        print("Error: %s" % str(e))
        client_socket.close()


if __name__ == "__main__":
    main()


