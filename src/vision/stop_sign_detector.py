import cv2

STOP_SIGN_HAAR = "stop_sign_haar.xml"

_cascade = cv2.CascadeClassifier(STOP_SIGN_HAAR)


# get direction of approach from color of stop sign
def get_stop_direction(stop_img):
    mean_color = cv2.mean(stop_img)
    #print("STOP BGR MEANS: ", mean_color)
    bgr_max = max(mean_color)
    bgr_min = min(mean_color)

    if bgr_max < 100:
        #print("BLACK/GRAY (north/south)")
        return 0
    else:
     # mean_color.index(bgr_max) == 0:
        #print("RED (east/west)")
        return 2
    # elif mean_color.index(bgr_max) == 1:
    #     print("GREEN")
    #     return 2
    # else:
    #     print("RED")
    #     return 3

def detect_stop_sign(img):
    global _cascade
    img_roi = img[0:img.shape[1], img.shape[0]/2:img.shape[0]]
    rects = _cascade.detectMultiScale(cv2.cvtColor(img_roi, cv2.COLOR_BGR2GRAY), 1.1, 5, cv2.CASCADE_SCALE_IMAGE , (40, 40))

    if len(rects) == 0:
        return (False, 0, [], None, -1)

    rects[:, 2:] += rects[:, :2]
    x1, y1, x2, y2 = rects[0]
    stop_img = img_roi[y1:y2, x1:x2]
    direction = get_stop_direction(stop_img)


    # shift back to original image
    #for i in range(len(rects)):
    rects[0][0] += img.shape[0]/2
    rects[0][2] += img.shape[0]/2


    # if len(rects) > 0:
    #stop_sign_detected = True
    x1, y1, x2, y2 = rects[0]
    normalized_distance = int(100 * ((y1 + y2) / 2.)/img.shape[1])
    # else:
    #     stop_img = None
    #     stop_sign_detected = False
    #     normalized_distance = 100
    return (True, normalized_distance, (x1, y1, x2, y2), stop_img, direction)
    #return (stop_sign_detected, normalized_distance, (x1, y1, x2, y2), stop_img)




