import numpy as np
import cv2
import serial
import math
from TargetDetector import TargetDetector


arduinoPath = '/'
BAUD_RATE = 9600

quitKey = 'q'

panAngle = 0 #TODO: fill in angle
tiltAngle = 0 #TODO:


maxPanAngle = 100 #TODO: and below
minPanAngle = 30
maxTiltAngle = 100
minTiltAngle = 30
centerThreshX = 10 #smaller nums = more strict aiming
centerThreshY = 10

def distance((x1,y1), (x2,y2)):
    return math.sqrt((x2-x1)**2 + (y2-y1)**2)


#takes in (x,y) tuple of object to look at
#updates panAngle and tilt global variables!
#returns True if object is centered (AKA FIRE!)
def mapServoPosition(objectCoords, imCenter, centerThreshX, centerThreshY):
    #TODO: want to try to use the links below. One to track the (largest) object and then use that info to move accordingly. Not sure if the track is necessary
    #TODO:https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
    #@NOTE: https://www.hackster.io/mjrobot/automatic-vision-object-tracking-5575c4
    
    global panAngle
    global tiltAngle
    x = objectCoords[0]
    y = objectCoords[1]
    xTargetCentered = False
    yTargetCentered = False
    
    if (x < imCenter[0] - centerThreshX):
        panAngle += 10 #TODO: update to proportional? maybe add derivative
        if panAngle > maxPanAngle:
            panAngle = maxPanAngle
    if (x > imCenter[0] + centerThreshX):
        panAngle -= 10 #TODO
        if panAngle < minPanAngle:
            panAngle = minPanAngle
    else:
        #object is 'x-centered' so like we can do whatever like make it move to a higher position to fire by telling the output we are centered
        xTargetCentered = True
    
    if (y < imCenter[1]  - centerThreshY):
        tiltAngle += 10 #TODO
        if tiltAngle > maxTiltAngle:
            tiltAngle = maxTiltAngle
    if (y > imCenter[1] + centerThreshY):
        tiltAngle -= 10 #TODO
        if tiltAngle < minTiltAngle:
            tiltAngle = minTiltAngle
    else:
        #object is 'y-centered' so we could do something here
        yTargetCentered = True
        
    return (xTargetCentered and yTargetCentered) #both targets centered?
    

def sendData(serialConnection, panAngle, tiltAngle, shoot):
    msg = str(panAngle) + " " + str(tiltAngle) + " " + ('y' if shoot else 'n') + '\r\n'
    serialConnection.write(msg.encode('utf-8')) #TODO: DO WE NEED STRING ENCODING TO READ???
    
    
    
    
    
def main():
    global centerThreshX, centerThreshY, panAngle, tiltAngle
    cap = cv2.VideoCapture(0)
    detector = TargetDetector(200,220)#180, 225)
    #ser = serial.Serial(arduinoPath, BAUD_RATE)
    
    while(True):
        #capture frame-by-frame
        ret, frame = cap.read()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        detector.colorImage = frame
        #!!!!!!!!!!!!! do stuff !!!!!!!!!
        #showIm = hsv[:,:,2]
        
        
        #binary = detector.findTarget(hsv[:,:,1],200,255) #TODO: maybe use this and tune it
        #sat = cv2.GaussianBlur(hsv[:,:,2],(5,5),cv2.BORDER_DEFAULT)
        sat = cv2.GaussianBlur(hsv[:,:,1],(5,5),cv2.BORDER_DEFAULT)
        obj = detector.getTargetCoordinates(sat)#hsv[:,:,1])
        if obj is not None:
            coords = obj[0]
            objSize = obj[1]
            
            imCenter = (frame.shape[1]/2, frame.shape[0]/2) #cols = x, rows = x
            shoot = mapServoPosition(coords, imCenter, centerThreshX, centerThreshY)
            #TODO: ONLY SHOOT WHEN WE ARE CENTERED AND THEN SEE A CIRCLE NEAR THE CENTER
            #sendData(ser, panAngle, tiltAngle, shoot) #arduino do your thing
        
        
        
        
        
        #display for debug
        #showIm = cv2.cvtColor(showIm, cv2.COLOR_HSV2BGR) #convert back to bgr for display
        cv2.imshow('frame', detector.debugImage)
        if cv2.waitKey(1) & 0xFF == ord(quitKey):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()


main()