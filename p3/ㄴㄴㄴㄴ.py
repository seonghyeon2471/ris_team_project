import cv2
import serial
import time

cap = cv2.VideoCapture(0)

ser = serial.Serial("/dev/ttyUSB0",460800)

while True:
    ret, frame = cap.read()
    if ret:
        cv2.imshow("f", frame)

    if cv2.waitKey(1) == 27:
        break
