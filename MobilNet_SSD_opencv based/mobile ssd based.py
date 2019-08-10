import multiprocessing as mp
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import RPi.GPIO as GPIO
import os
import time
import sys

DELAY = 2
#initialize led pins
POWER_LED_PIN = 2
DETECT_PIN = 3#for person detect

#init settings
GPIO.setmode(GPIO.BOARD)
GPIO.setup(POWER_LED_PIN, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(DETECT_PIN, GPIO.OUT, initial=GPIO.LOW)

#Load caffe model
prototxt = "MobileNetSSD_deploy.prototxt"
weights = "MobileNetSSD_deploy.caffemodel"
thr = .8

# Labels of Network.
classNames = {15: 'person'}

#Load the Caffe model 
net = cv2.dnn.readNetFromCaffe(prototxt, weights)

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

    #frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    frame_resized = cv2.resize(frame,(300,300)) # resize frame for prediction
    heightFactor = frame.shape[0]/300.0
    widthFactor = frame.shape[1]/300.0

    blob = cv2.dnn.blobFromImage(frame_resized, 0.007843, (300, 300), (127.5, 127.5, 127.5), False)
    #Set to network the input blob 
    net.setInput(blob)
    #Prediction of network
    detections = net.forward()

    
    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2] #Confidence of prediction 
        if confidence > thr and 15 = int(detections[0, 0, i, 1]):#15 -> person, detections[0,0,i,1] gives class id
            GPIO.output(DETECT_PIN, GPIO.HIGH)
            x = int(detections[0, 0, i, 3] * cols * widthFactor)
            y = int(detections[0, 0, i, 5] * cols * widthFactor)
            cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
            l = min((x+w)/2,l)
            r = max((x+w)/2,r)

    #changing scaling factor
    r = r/320 * 180
    l = l/320 * 180
    
    key = cv2.waitKey(1000)
    cv2.destroyAllWindows()
    rawCapture.truncate(0)
    GPIO.output(DETECT_PIN, GPIO.LOW)
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
        time.sleep(DELAY)
        l1, r1 = imgProcess(0)

        #mid position
        cam.ChangeDutyCycle(7.5)
        time.sleep(DELAY)
        l2, r2 = imgProcess(60)

        #right position
        cam.ChangeDutyCycle(9)
        time.sleep(DELAY)
        l3, r3 = imgProcess(120)
        
        #update shared variables values
        left.value =  min(l1, l2, l3) * (1/18) + 2.5
        right.value = max(r1, r2, r3) * (1/18) + 2.5
        flag.value = 1
        time.sleep(DELAY)


if __name__ == '__main__':
    
    #start fan process
    mp.Process(target=rotateFan, args=(flag, left, right)).start()
    time.sleep(1)
    cam_rotateNProess(flag, left, right)
     
