#opens up a webcam feed so you can then test your classifer in real time
#using detectMultiScale
import numpy as np
import cv2

CONSECUTIVE_BUFFER_SIZE = 15
CONSECUTIVE_THRESHOLD = 0.50

def drawBoxes(rects, img):
    for x1, y1, x2, y2 in rects:
        cv2.rectangle(img, (x1, y1), (x2, y2), (127, 255, 0), 2)

def drawText(img, message):
   cv2.putText(img, message, (50,50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 1)

def detectLines(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    (thresh, img_bw) = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY)
    img_bw = cv2.GaussianBlur(img_bw, (3, 3), 0)
    #gray = cv2.equalizeHist(gray)
    cv2.imshow("AutoCross Car blur", img_bw)

    # smooth and canny edge detection
    canny = cv2.Canny(img_bw, 50, 200)
    #canny = cv2.Canny(gray, 50, 200)

    #ret, thresh = cv2.threshold(equ, 250, 255, cv2.THRESH_BINARY)

    lines = cv2.HoughLinesP(canny, 1, np.pi/180, 100, minLineLength = 10, maxLineGap = 10)

    if (lines is None or len(lines) == 0):
        return [], img

    return lines, img

def detectStopSign(cascade, img):
    rects = cascade.detectMultiScale(img, 1.3, 4, cv2.cv.CV_HAAR_SCALE_IMAGE, (20, 20))

    if len(rects) == 0:
        return [], img
    rects[:, 2:] += rects[:, :2]

    return rects, img

def detectLanes(lines):
    print lines[:,0]
    return lines

def drawLanes(lines, img):
    for x1,y1,x2,y2 in lines[:,0]:
        cv2.line(img,(x1, y1),(x2, y2), (0, 255, 0), 2, cv2.CV_AA) #rem out if u want to used  polykines
        #pts = np.array([[x1, y1 ], [x2 , y2 ] ], np.int32)
        #cv2.polylines(img, [pts], True, (0, 255, 255))

def main():
    cap = cv2.VideoCapture(0)
    cap.set(3, 700)
    cap.set(4, 600)

    cascade = cv2.CascadeClassifier("frontal_stop_sign_cascade.xml")

    check_buffer_count = []

    while(True):
        ret, img = cap.read()

        rects, img = detectStopSign(cascade, img)
        drawBoxes(rects, img)

        lines, img = detectLines(img)

        #TODO: process lines to detect lanes via length, and location
        if len(lines) != 0:
            lines = detectLanes(lines)
            drawLanes(lines, img)

        check_buffer_count += [len(rects)]

        if (len(check_buffer_count) >= CONSECUTIVE_BUFFER_SIZE):
            check_buffer_count = check_buffer_count[1:]
            #check percentage
            threshold = float(sum(check_buffer_count))/float(len(check_buffer_count))
            print "Running Threshold: " + str(threshold)
            if (threshold >= CONSECUTIVE_THRESHOLD):
                #sign detected
                drawText(img, "Status: SEND STOP SIGNAL")
            else:
                drawText(img, "Status: ")


        drawText(img, "Status: ")
        cv2.imshow("AutoCross Car Control", img)

        if(cv2.waitKey(1) & 0xFF == ord('q')):
            break

main()
