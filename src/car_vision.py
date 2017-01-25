#opens up a webcam feed so you can then test your classifer in real time
#using detectMultiScale
import numpy as np
import cv2
import time
from picamera.array import PiRGBArray
from picamera import PiCamera

CONSECUTIVE_BUFFER_SIZE = 15
CONSECUTIVE_THRESHOLD = 0.50
DEFAULT_RESOLUTION_WIDTH = 600
DEFAULT_RESOLUTION_HEIGHT = 600

HOUGH_LINE_RHO = 1
HOUGH_LINE_THRESHOLD = 75
HOUGH_LINE_MIN_LENGTH = 50
HOUGH_LINE_MAX_GAP = 10

def drawBoxes(rects, img):
    for x1, y1, x2, y2 in rects:
        cv2.rectangle(img, (x1, y1), (x2, y2), (127, 255, 0), 2)

def drawText(img, message):
   cv2.putText(img, message, (50,50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 1)

def detectLines(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    (thresh, img_bw) = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
    img_bw = cv2.GaussianBlur(img_bw, (3, 3), 0)
    #gray = cv2.equalizeHist(gray)

    # smooth and canny edge detection
    canny = cv2.Canny(img_bw, 50, 200)

	#minLineLength = 10, maxLineGap = 10)
    lines = cv2.HoughLinesP(canny, HOUGH_LINE_RHO, np.pi/180, HOUGH_LINE_THRESHOLD, minLineLength = HOUGH_LINE_MIN_LENGTH, maxLineGap = HOUGH_LINE_MAX_GAP)

    if (lines is None or len(lines) == 0):
        return [], img

    return lines, img

def detectStopSign(cascade, img):
    rects = cascade.detectMultiScale(img, 1.3, 4, cv2.CASCADE_SCALE_IMAGE , (20, 20))

    if len(rects) == 0:
        return [], img
    rects[:, 2:] += rects[:, :2]

    return rects, img

def detectLanes(lines):
    print lines[:,0]
    
    lines = lines[:,0]
    filteredLines = horizontalFilter(lines, 0.40 * DEFAULT_RESOLUTION_HEIGHT, DEFAULT_RESOLUTION_HEIGHT)
    
    return filteredLines
    
def horizontalFilter(lines, minValue, maxValue):
	linesFiltered = []
	print "LINES",len(lines)
	for i in range(len(lines)):
		x1, y1, x2, y2 = lines[i]
		if between(y1, minValue, maxValue) and between(y2, minValue, maxValue):
			linesFiltered += [[x1, y1, x2, y2]]
			
	return linesFiltered
		
	
def between(value, minValue, maxValue):
	return value > minValue and value < maxValue	

def drawLanes(lines, img):
	for i in range(len(lines)):
		x1 = lines[i][0]
		y1 = lines[i][1] 
		x2 = lines[i][2]
		y2 = lines[i][3]
		cv2.line(img,(x1, y1),(x2, y2), (0, 255, 0), 2) #rem out if u want to used  polykines

def determineStopSignal(stop_sign_buffer, rects):
	stop_sign_buffer += [len(rects)]

	if (len(stop_sign_buffer) >= CONSECUTIVE_BUFFER_SIZE):
		stop_sign_buffer = stop_sign_buffer[1:]
		#check percentage
		threshold = float(sum(stop_sign_buffer))/float(len(stop_sign_buffer))
		print "Running Threshold: " + str(threshold)
		if (threshold >= CONSECUTIVE_THRESHOLD):
			return True
		else:
			return False

def startVision():    
    camera = PiCamera()
    camera.resolution = (DEFAULT_RESOLUTION_WIDTH, DEFAULT_RESOLUTION_HEIGHT)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(DEFAULT_RESOLUTION_WIDTH, DEFAULT_RESOLUTION_HEIGHT))
    
    #let camera start up
    time.sleep(0.5)

    stopSignCascade = cv2.CascadeClassifier("frontal_stop_sign_cascade.xml")
    
    stop_sign_buffer_count = [0]
    
    frameCount = 0

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

        img = frame.array
        rawCapture.truncate(0)

        rects, img = detectStopSign(stopSignCascade, img)
        
        stopSignDetected = determineStopSignal(stop_sign_buffer_count, rects)
       
        drawBoxes(rects, img)

        lines, img = detectLines(img)
        
        if stopSignDetected:        	
			drawText(img, "Status: STOP SIGN DETECTED dist=" + str(stopSignDistance))

        #TODO: process lines to detect lanes via length, and location
        if len(lines) != 0:
            lines = detectLanes(lines)
            drawLanes(lines, img)
            
        frameCount += 1
        drawText(img, "Status: " + str(frameCount))
        cv2.imshow("AutoCross Car Control", img)

        if(cv2.waitKey(1) & 0xFF == ord('q')):
			cv2.destroyAllWindows()
			#I am really not sure why this works... I need to commit
			for i in range(4):
				cv2.waitKey(1)
			break

if __name__ == "__main__": startVision()
