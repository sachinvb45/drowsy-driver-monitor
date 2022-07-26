#Importing OpenCV Library for basic image processing functions
import cv2
# Numpy for array related functions
import numpy as np
# Dlib for deep learning based Modules and face landmark detection
import dlib
#face_utils for basic operations of conversion
from imutils import face_utils
import RPi.GPIO as GPIO  # Import Raspberry Pi GPIO library
from time import sleep  # Import the sleep function from the time module
GPIO.setwarnings(False)  # Ignore warning for now
GPIO.setmode(GPIO.BOARD)  # Use physical pin numbering
GPIO.setup(8, GPIO.OUT, initial=GPIO.LOW)
buzz=13
GPIO.setup(buzz,GPIO.OUT, initial=GPIO.LOW)
sup=100
GPIO.setup(33,GPIO.OUT)
GPIO.setup(35,GPIO.OUT, initial=GPIO.HIGH)

p=GPIO.PWM(33,1000)
p.start(100)


def fast(sup):
    if sup<100:
        sup+=10
    if sup>=100:
        sup=100
    p.ChangeDutyCycle(sup)

def slow(sup):
    if sup<100:
        sup=sup-10
    if sup<=0:
        sup=0
    p.ChangeDutyCycle(sup)
#     sleep(0.1)#Motor will run at slow speed
#     GPIO.output(33,True)




#Initializing the camera and taking the instance
# For IP camera
# cap = cv2.VideoCapture("http://192.168.208.110:8080/video")
# For Raspberry Pi camera or USB camera
cap = cv2.VideoCapture(0)

#Initializing the face detector and landmark detector
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")

#status marking for current state
sleep = 0
drowsy = 0
active = 0
status=""
color=(0,0,0)

def compute(ptA,ptB):
	dist = np.linalg.norm(ptA - ptB)
	return dist

def blinked(a,b,c,d,e,f):
	up = compute(b,d) + compute(c,e)
	down = compute(a,f)
	ratio = up/(2.0*down)

	#Checking if it is blinked
	if(ratio>0.23 ):
		return 2
	elif(ratio>0.18 and ratio<=0.23):
		return 1
	else:
		return 0
	

face_frame=0 
while True:
    _, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    faces = detector(gray)
    #detected face in faces array
    for face in faces:
        x1 = face.left()
        y1 = face.top()
        x2 = face.right()
        y2 = face.bottom()

        face_frame= frame.copy()
        cv2.rectangle(face_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        landmarks = predictor(gray, face)
        landmarks = face_utils.shape_to_np(landmarks)
        left_blink = blinked(landmarks[36],landmarks[37], 
        	landmarks[38], landmarks[41], landmarks[40], landmarks[39])
        right_blink = blinked(landmarks[42],landmarks[43], 
        	landmarks[44], landmarks[47], landmarks[46], landmarks[45])
        if(left_blink==0 or right_blink==0):
        	# sleep+=1
            sleep+=1
            drowsy=0
            active=0
            if(sleep>3):
                sup=sup-10
                status="sleeping"
                print("sleeping")
                color=(255,0,0)
                GPIO.output(8, GPIO.HIGH)
                GPIO.output(buzz,GPIO.HIGH)
                slow(sup)
                print ("Buzzer is Beeping")# Turn on
                
        elif(left_blink==1 or right_blink==1):
            sleep=0
            active=0
            drowsy+=1
            if(drowsy>2):
                sup=sup-10
                status="Drowsy"
                print("Drowsy")
                color=(0,0,255)
                GPIO.output(buzz,GPIO.HIGH)
                slow(sup)
                
        else:
            drowsy=0
            sleep=0
            active+=1
            if(active>6):
                if sup<=0:
                    sup=10
                status="active"
                print("active")
                color=(0,255,0)
                GPIO.output(8, GPIO.LOW)
                GPIO.output(buzz,GPIO.LOW)
                p.ChangeDutyCycle(100)
                
        cv2.putText(frame,status,(100,100),cv2.FONT_HERSHEY_SIMPLEX,1.2,color,3)
        for n in range(0,68):
            (x,y)=landmarks[n]
            cv2.circle(face_frame,(x,y),1,(255,255,255),-1)
    cv2.imshow("Frame",frame)
    cv2.imshow("Result of detector",face_frame)
    key=cv2.waitKey(1)
    if (key==27):
        break