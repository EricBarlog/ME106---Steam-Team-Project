###################################################################################################
# Group Project Steam Team
# ME106 Dr. Jiang
# Eric Barlog, David Jefferson, Jeremy Mendoza
###################################################################################################
# This project uses the Pimoroni Pan Tilt Hat, Raspberry Pi 4, 1605 OLED Screen. ADXL345
# The vision library is OpenCV provided by Intel's opensource vision library
###################################################################################################

# initialize libraries
import cv2, sys, time, os, pantilthat, busio, board, adafruit_adxl34x, subprocess
from pantilthat import *
from board import SCL, SDA
from oled_text import OledText

### Accelorometer stuff
Acceli2c = busio.I2C(SCL, SDA)
accelerometer = adafruit_adxl34x.ADXL345(Acceli2c, 0x53) # pins and i2c ID

### OLED stuff
OLEDi2c = busio.I2C(SCL, SDA)
# Create the display, pass its pixel dimensions
oled = OledText(OLEDi2c, 128, 64)

# Update OLED to initial conditions
cmd = "hostname -I | cut -d\' \' -f1"
IP = subprocess.check_output(cmd, shell = True )
oled.text('IP' + str(IP), 1)  # Line 1

### Vision Camera Stuff
# load BCM V4l2 driver
os.system('sudo modprobe bcm2835-v4l2')
# set framerate of rpi 4 cam
os.system('v4l2-ctl -p 40')

# initial position of camera on activation (range is 0-180)
camPan = 90
camTilt = 40

# tilts and pans camera to start position
pan(camPan-90)
tilt(camTilt-90)

# establish camera framesize
FRAME_W = 320
FRAME_H = 200

# recoridng window of the camera frame 
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W);
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H);
# delay for capture to not overwhelm the pi
time.sleep(2) 

# routes the haar cascade path and local binary pattern
cascPath = '/usr/share/opencv/lbpcascades/lbpcascade_frontalface.xml'
faceCascade = cv2.CascadeClassifier(cascPath)

# main while loop of facial tracking
while True:
    # record each frame of the camera
    rec, frame = cap.read()
    # rotates camera frame 180* so inmage is displayed right side up
    frame = cv2.flip(frame, -1)
    
    # double check camera is working 
    if rec == False:
      print("check camera connection")
      continue

    # change to grey scale to make tracking more accurate
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist( gray )

    # track face for these frames, scaleFactor 1.05-1.1, minNeighbors 3 per box, minSize anything smaller is ignored
    faces = faceCascade.detectMultiScale(frame, 1.1, 3, 0, (10, 10))
    
    # creates rectangle around face being tracked
    for (x, y, w, h) in faces:
        # puts frame around the face
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 1)
        
        # find center of face
        x = x + (w/2)
        y = y + (h/2)

        # move to center of face and track
        turn_x  = float(x - (FRAME_W/2))
        turn_y  = float(y - (FRAME_H/2))

        # conver to percentage offset
        turn_x  /= float(FRAME_W/2)
        turn_y  /= float(FRAME_H/2)

        # scale offset to degrees for servos to understand, movement step size
        turn_x   *= 2.5 # VFOV
        turn_y   *= 2.5 # HFOV
        camPan  += -turn_x
        camTilt += turn_y

        # print position to console as it updates
        print('Pan Pos: ' + str(camPan-90), 'Tilt Pos: ' + str(camTilt-90))
        x1, y1, z1 = accelerometer.acceleration
        print('X-Axis: ' + str(x1))  # Line 1
        print('Y-Axis: ' + str(y1))  # Line 2
        print('Z-Axis: ' + str(z1))  # Line 3

        # virtual hardstops for servos
        camPan = max(0,min(180,camPan))
        camTilt = max(0,min(180,camTilt))

        # update servos
        pan(int(camPan-90))
        tilt(int(camTilt-90))

        break
    
    # resize frame for viewing and display
    frame = cv2.resize(frame, (1248, 702))
    frame = cv2.flip(frame, 1)
   
    # display video
    cv2.imshow("Smile! You're on camera!", frame)

    # quits tracking when 'Q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
# stop recording and release connection
video_capture.release()
cv2.destroyAllWindows()