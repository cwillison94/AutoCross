import cv2

display_on = False

def set_display_on(on):
    global display_on
    display_on = on

def draw_text(img, message, location = (50, 50)):
    global display_on
    if display_on:
        cv2.putText(img, message, location, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 1)

    return img

def draw_lanes(img, left_lane, right_lane):
    global display_on
    if display_on and left_lane is not None and right_lane is not None:
        # draw_text(img, "steering %.1f " % steering_output, (50, 300))
        cv2.line(img, (left_lane[0], left_lane[1]), (left_lane[2], left_lane[3]), (0, 0, 255), 5)
        cv2.line(img, (right_lane[0], right_lane[1]), (right_lane[2], right_lane[3]), (0, 0, 255), 5)

    return img

def draw_rectangle(img, rect):
    global display_on
    if display_on and len(rect) == 4:
        cv2.rectangle(img, (rect[0], rect[1]), (rect[2], rect[3]), (255, 255, 0), 5)

    return img

def draw_steering_output(img, steering_output):
    global display_on
    if display_on:
        draw_text(img, "steering %.1f " % steering_output, (50, 300))

    return img

def draw_line(img, line):
    global display_on
    if display_on and line is not None:
        cv2.line(img, (line[0], line[1]), (line[2], line[3]),  (255, 0, 0), 5)
    
    return img

def draw_midline(img, car_midline):
    global display_on
    if display_on:
        cv2.line(img, (car_midline, 0), (car_midline, img.shape[1]),(0, 255,0), 5)

    return img
