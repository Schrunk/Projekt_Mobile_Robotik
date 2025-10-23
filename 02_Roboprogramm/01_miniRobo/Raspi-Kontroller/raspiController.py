#!/usr/bin/env python3

import RPi.GPIO as GPIO
import socket
import time

# --- UDP Konfiguration ---
UDP_IP = "192.168.178.141"  # IP deines Ubuntu-Hosts
UDP_PORT = 9999           # Port des micro-ROS-Agenten
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# --- GPIO Pins ---
PIN_UP = 17
PIN_DOWN = 27
PIN_LEFT = 22
PIN_RIGHT = 23

GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN_UP, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(PIN_DOWN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(PIN_LEFT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(PIN_RIGHT, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def send_button(button_name):
    try:
        sock.sendto(button_name.encode(), (UDP_IP, UDP_PORT))
        print(f"Sent: {button_name}")
    except Exception as e:
        print(f"Error sending {button_name}: {e}")

try:
    print("Button listener started. Press Ctrl+C to exit.")
    while True:
        if not GPIO.input(PIN_UP):
            send_button("up")
            time.sleep(0.2)
        if not GPIO.input(PIN_DOWN):
            send_button("down")
            time.sleep(0.2)
        if not GPIO.input(PIN_LEFT):
            send_button("left")
            time.sleep(0.2)
        if not GPIO.input(PIN_RIGHT):
            send_button("right")
            time.sleep(0.2)
        time.sleep(0.01)

finally:
    GPIO.cleanup()
    sock.close()

