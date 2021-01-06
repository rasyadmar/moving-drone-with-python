import serial
from time import sleep
import sys


COM = input("Arduino Port: ") #'COM5' /dev/ttyACM0 (Linux)
BAUD = 9600

ser = serial.Serial(COM, BAUD, timeout = .1)
print('Waiting for device')
sleep(3)
print(ser.name)


while True:
	val = str(ser.readline().decode().strip())#Capture serial output as a decoded string
	valA = val.split(",")
	# print(valA, end="\r", flush=True)
	print(valA)