import numpy as np
import cv2

class TargetDetector:
    """Detector for our Bucket"""
    
    def __init__(self, lowerBound, upperBound):
        #bounds for color matcher
        self.lowerBound = lowerBound
        self.upperBound = upperBound
        self.output = []
        self.debugImage = None
        self.colorImage = None
    
    
    
    #image: single channel image in which to search for the target
    #return: masked image based on upper and lower bounds
    def findTarget(self, image, lower, upper):
        mask = cv2.inRange(image, lower, upper)
        
        return mask
        
    def filterBinaryNoise(self, image):
        #opening to get rid of stray pixels
        kernel0 = cv2.getStructuringElement(cv2.MORPH_RECT, (9,9))
        opening = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel0)
        
        #closing to fill blob correctly
        kernel1 = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel1)
        
        return closing

    def findCentroidOverall(self, binaryIm):
        # calculate moments of binary image
        M = cv2.moments(binaryIm, binaryImage=True)
         
        # calculate x,y coordinate of center
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            cX, cY = 0, 0
        
        return cX, cY
        
    #takes in binaryImage (from threshold)
    #output: maxArea, output[]
    #   maxArea: largest area of contours
    #   output: list of centroids and areas -- ((cX,cY),area)
    def findMultipleCentroids(self, binaryIm):
        #output = []
        self.output = []
        im2, contours, hierarchy = cv2.findContours(binaryIm,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        
        #debug the contours
        img = np.zeros((binaryIm.shape[0], binaryIm.shape[1], 3))
        cv2.drawContours(img, contours, -1, (0,255,0), 3)
        cv2.imshow("contours", img)
        
        maxArea = 0
        for c in contours:
            #calc area
            area = cv2.contourArea(c)
            if area > maxArea:
                maxArea = area
                
        
            # calculate moments for each contour
            M = cv2.moments(c)
         
            # calculate x,y coordinate of center
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                cX, cY = 0, 0
            self.output.append(((cX,cY), area))
        return maxArea, self.output
    
    '''Uses Hough Circle Transform to find circles, then picks the average of them'''
    #returns coords of object by averaging found circles or -1,-1 if no object found
    def houghCircleFinder(self, frame):
        circles = cv2.HoughCircles(frame, cv2.HOUGH_GRADIENT, 1, 20,
                  param1=80,#80,#100,
                  param2=70,#30,
                  minRadius=0,
                  maxRadius=0)
        #debug   
        img = np.zeros((frame.shape[0], frame.shape[1], 3))
        img = self.colorImage
        
        avgY = -1
        avgX = -1
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0,:]:
                cv2.circle(img,(i[0],i[1]),i[2],(0,255,0),2)
                cv2.circle(img,(i[0],i[1]),2,(0,0,255),3)
                avgY = i[1]
                avgX = i[0]
            avgY /= len(circles)
            avgX /= len(circles)
        cv2.imshow('hough', img)
        
        return avgX, avgY
        
    
    
    """Gets bucket location from image (hopefully in hsv format)"""
    #returns None if no objects found or ((x,y), area) of largest object found
    def getTargetCoordinates(self, intensity_image):
        binary = self.findTarget(intensity_image, self.lowerBound, self.upperBound) #image[:,:,1]
        #binary = self.filterBinaryNoise(binary)
        #do something about finding blob of object (something like SimpleBlobDetector) #TODO?
        #TODO: may need to do some hough circle detection instead depending on how the target looks
        #cX, cY = findCentroidOverall(showIm)
        maxArea, centroids = self.findMultipleCentroids(binary)
        showIm = np.zeros((binary.shape[0], binary.shape[1], 3))
        #do some things to allow color on our binary output
        showIm[:,:,0] = binary
        showIm[:,:,1] = binary
        showIm[:,:,2] = binary
        
        #testing hough circles
        circleAvgX, circleAvgY = self.houghCircleFinder(intensity_image)
        if not(circleAvgX == -1 or circleAvgX == -1):
            circleSeen = True
        else:
            circleSeen = False
        
        
        #find object by blob area
        largestObject = None #((x,y), area). None if we don't find anything
        for c in centroids:
            cX = c[0][0]
            cY = c[0][1]
            area = c[1]
            if (area == maxArea): #TODO: do we want only the largest contour?
                largestObject = c
                cv2.circle(showIm, (cX,cY), 5, (0, 0, 255), -1)
                cv2.putText(showIm, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        self.debugImage = showIm
        
        return largestObject
        
        
        