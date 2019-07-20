import multiprocessing as mp
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import RPi.GPIO as GPIO
import os
import time
import sys

#init settings
GPIO.setmode(GPIO.BOARD)
#full body classifier
pathtobody = os.path.join(sys.path[0],'case.xml')
body_cascade = cv2.CascadeClassifier(pathtobody)

#init variiables for all process
left = mp.Value('d', 2.5)#for 0 angle
right = mp.Value('d', 12.5)#for 180 angle
flag = mp.Value('i', 1)#1 for unlock & 0 for lock

#fan rotating function
def rotateFan(flag, left, right):
    #init values
    fanServoPIN = 16
    GPIO.setup(fanServoPIN, GPIO.OUT)
    fan = GPIO.PWM(fanServoPIN, 50);
    fan.start(0)
    leftAngle = 2.5
    rightAngle = 12.5

    while True:
        if flag.value == 1:
            leftAngle = left.value
            rightAngle = right.value
            flag.value = 0
            
        fan.ChangeDutyCycle(leftAngle)
        time.sleep(2)
        fan.ChangeDutyCycle(rightAngle)
        time.sleep(2)

#takeing pictures & processing function
def imgProcess(angle):
    #init values
    frame = None
    l = 1000
    r = -1000
    
    with PiCamera() as camera:
        camera.resolution = (320, 240)
        camera.framerate = 30
        rawCapture = PiRGBArray(camera, size=(320, 240))
        time.sleep(.5)
        camera.capture(rawCapture, format="bgr")
        frame = rawCapture.array

    frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    bodies = body_cascade.detectMultiScale(frame, 1.1, 5)
    for (x,y,w,h) in bodies:
        cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
        l = min((x+w)/2,l)
        r = max((x+w)/2,r)

    #changing scaling factor
    r = r/320 * 180
    l = l/320 * 180
    
    key = cv2.waitKey(1000)
    cv2.destroyAllWindows()
    rawCapture.truncate(0)

    return l + angle, r + angle

#rotating camera fan fuunction
def cam_rotateNProess(flag, left, right):
    #init values
    camServoPIN = 11
    GPIO.setup(camServoPIN, GPIO.OUT)
    cam = GPIO.PWM(camServoPIN, 50) # for PWM with 50Hz
    cam.start(5)

    while True:
        #left position
        cam.ChangeDutyCycle(5.5)
        time.sleep(2)
        l1, r1 = imgProcess(0)

        #mid position
        cam.ChangeDutyCycle(7.5)
        time.sleep(2)
        l2, r2 = imgProcess(60)

        #right position
        cam.ChangeDutyCycle(9)
        time.sleep(2)
        l3, r3 = imgProcess(120)
        
        #update shared variables values
        left.value =  min(l1, l2, l3) * (1/18) + 2.5
        right.value = max(r1, r2, r3) * (1/18) + 2.5
        flag.value = 1
        time.sleep(1)



if __name__ == '__main__':
    
    #start fan process
    mp.Process(target=rotateFan, args=(flag, left, right)).start()
    time.sleep(1)
    cam_rotateNProess(flag, left, right)
     
