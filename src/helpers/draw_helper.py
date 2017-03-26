import cv2

display_on = False

def set_display_on(on):
    display_on = on

def draw_text(img, message, location = (50, 50)):
    if display_on:
        cv2.putText(img, message, location, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 1)

def draw_lanes(img, left_lane, right_lane, steering_output):
    if display_on:
        draw_text(img, "steering %.1f " % steering_output, (50, 300))
        cv2.line(img, (left_lane[0], left_lane[1]), (left_lane[2], left_lane[3]), (0, 0, 255), 5)
        cv2.line(img, (right_lane[0], right_lane[1]), (right_lane[2], right_lane[3]), (0, 0, 255), 5)

def draw_midline(img, car_midline):
    if display_on:
        cv2.line(img, (car_midline, 0), (car_midline, img.shape[1]),(0, 255,0), 5)
