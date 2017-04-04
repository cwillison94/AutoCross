import cv2

STOP_SIGN_HAAR = "stop_sign_haar.xml"

_cascade = cv2.CascadeClassifier(STOP_SIGN_HAAR)

def detect_stop_sign(img):
    global _cascade
    img_roi = img[0:img.shape[1], img.shape[0]/2:img.shape[0]]
    rects = _cascade.detectMultiScale(cv2.cvtColor(img_roi, cv2.COLOR_BGR2GRAY), 1.1, 5, cv2.CASCADE_SCALE_IMAGE , (25, 25))

    if len(rects) == 0:
        return (False, 100, [])

    rects[:, 2:] += rects[:, :2]

    # shift back to original image
    for i in range(len(rects)):
        rects[i][0] += img.shape[0]/2
        rects[i][2] += img.shape[0]/2

        #rects[i][1] += img.shape[1]/2
        #rects[i][3] += img.shape[1]/2

    if len(rects) > 0:
        stop_sign_detected = True
        x1, y1, x2, y2 = rects[0]
        normalized_distance = int(100 * ((y1 + y2) / 2.)/img.shape[1])
    else:
        stop_sign_detected = False
        normalized_distance = 100

    return (stop_sign_detected, normalized_distance, (x1, y1, x2, y2))


