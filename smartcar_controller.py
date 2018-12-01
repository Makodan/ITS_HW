#!/usr/bin/env python

import socket
import time
from picamera import PiCamera
import RPi.GPIO as GPIO
import sys
from threading import Thread, Lock
import VL53L0X
import paho.mqtt.client as mqtt


# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("smartcar/commands")


# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print(msg.topic + " " + str(msg.payload))
    set_directions(str(msg.payload))


UDP_IP = '192.168.0.111'  # The remote host
UDP_PORT = 50005  # The same port as used by the server
# HOST = sys.argv[1]
# PORT = sys.argv[2]
IsActive = True
LB_pin = 33
LA_pin = 31
RB_pin = 37
RA_pin = 35

# Create a VL53L0X object
tof = VL53L0X.VL53L0X()
# Start ranging
tof.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect("192.168.0.109", 1883, 60)
client.loop_start()


def main():
    global IsActive
    global sendActive
    try:
        Thread(target=camera_stream, args=(), name="CameraStream").start()
        Thread(target=measure_range, args=[tof], name="Ranging").start()
        init_pins()
        inp = ''

        while inp != 'x':
            print('Input')
            inp = raw_input()
            if inp == 'w':
                set_directions('forward')
            if inp == 'l':
                set_directions('left')
            if inp == 'r':
                set_directions('right')
            if inp == 'b':
                set_directions('backward')
            if inp == 's':
                set_directions('stop')

        IsActive = False
        sendActive = False
        GPIO.cleanup()
        tof.stop_ranging()
        client.loop_stop()

    except KeyboardInterrupt:
        IsActive = False
        sendActive = False
        GPIO.cleanup()
        tof.stop_ranging()
        client.loop_stop()

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
        print("[Info] Stream stopped")

    except Exception as e:
        print("Error: %s" % str(e))
        client_socket.close()


def init_pins():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(LA_pin, GPIO.OUT)
    GPIO.setup(LB_pin, GPIO.OUT)
    GPIO.setup(RA_pin, GPIO.OUT)
    GPIO.setup(RB_pin, GPIO.OUT)


def set_directions(direction):
    if direction == 'forward':
        GPIO.output(LA_pin, GPIO.HIGH)
        GPIO.output(LB_pin, GPIO.LOW)
        GPIO.output(RA_pin, GPIO.HIGH)
        GPIO.output(RB_pin, GPIO.LOW)

    elif direction == 'left':
        GPIO.output(LA_pin, GPIO.LOW)
        GPIO.output(LB_pin, GPIO.HIGH)
        GPIO.output(RA_pin, GPIO.HIGH)
        GPIO.output(RB_pin, GPIO.LOW)

    elif direction == 'right':
        GPIO.output(LA_pin, GPIO.HIGH)
        GPIO.output(LB_pin, GPIO.LOW)
        GPIO.output(RA_pin, GPIO.LOW)
        GPIO.output(RB_pin, GPIO.HIGH)

    elif direction == 'backward':
        GPIO.output(LA_pin, GPIO.LOW)
        GPIO.output(LB_pin, GPIO.HIGH)
        GPIO.output(RA_pin, GPIO.LOW)
        GPIO.output(RB_pin, GPIO.HIGH)

    # STOP
    else:
        GPIO.output(LA_pin, GPIO.LOW)
        GPIO.output(LB_pin, GPIO.LOW)
        GPIO.output(RA_pin, GPIO.LOW)
        GPIO.output(RB_pin, GPIO.LOW)


def measure_range(tof):
    while IsActive:
        timing = tof.get_timing()
        sum_distance = 0
        for count in range(1, 5):
            distance = tof.get_distance()
            if distance > 0:
                sum_distance += distance
            time.sleep(timing / 1000000.00)
        avg_distance = (sum_distance / 5)
        print (avg_distance)
        if avg_distance > 2000:
            avg_distance = 2000
        client.publish("smartcar/distance", avg_distance)
        time.sleep(0.05)


if __name__ == "__main__":
    main()
