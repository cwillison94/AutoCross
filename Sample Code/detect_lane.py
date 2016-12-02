import cv2
import numpy as np

cap = cv2.VideoCapture('test.avi')
cap.open('test.avi')

print cap.isOpened()

while (cap.isOpened()): 
    ret, img = cap.read()
    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    equ = cv2.equalizeHist(gray)
      
    # smooth and canny edge detection
    edge = cv2.Canny(equ, 200, 255, 3)
    ret, thresh = cv2.threshold(equ, 250, 255, cv2.THRESH_BINARY)
    
    lines = cv2.HoughLinesP(thresh, cv2.HOUGH_PROBABILISTIC, np.pi/180, 20, minLineLength = 10, maxLineGap = 2)
    for x1,y1,x2,y2 in lines[:,0]:
        cv2.line(img,(x1, y1),(x2, y2), (0, 255, 0), 2, cv2.LINE_AA) #rem out if u want to used  polykines  
        #pts = np.array([[x1, y1 ], [x2 , y2 ] ], np.int32)
        #cv2.polylines(img, [pts], True, (0, 255, 255))
        
        
    cv2.imshow('video', img)
    cv2.imshow('equ', equ)
    cv2.imshow('edge', edge)
    if cv2.waitKey(1) & 0xFF == ord('q'):
      break

cap.released()
cv2.destroyAllWindow()
          
