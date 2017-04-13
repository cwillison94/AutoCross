import cv2

display_on = True

def set_display_on(on):
    global display_on
    display_on = on


def _scale_line(x1, y1, x2, y2, height):
    road_horizon = 300
    # scale the farthest point of the segment to be on the drawing horizon
    if x1 == x2:
        if y1 < y2:
            y1 = road_horizon
            y2 = height
            return x1, y1, x2, y2
        else:
            y2 = road_horizon
            y1 = height
            return x1, y1, x2, y2
    if y1 < y2:
        m = (y1-y2)/(x1-x2)
        x1 = ((road_horizon-y1)/m) + x1
        y1 = road_horizon
        x2 = ((height-y2)/m) + x2
        y2 = height
    else:
        m = (y2-y1)/(x2-x1)
        x2 = ((road_horizon-y2)/m) + x2
        y2 = road_horizon
        x1 = ((height-y1)/m) + x1
        y1 = height
    return int(x1), int(y1), int(x2), int(y2)

def draw_text(img, message, location = (50, 50)):
    global display_on
    if display_on:
        cv2.putText(img, message, location, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 1)

    return img

def draw_lanes(img, left_lane, right_lane):
    global display_on
    if display_on and left_lane is not None and right_lane is not None:

        # scale line for better visual representation
        h = img.shape[0]
        left_lane = _scale_line(left_lane[0], left_lane[1], left_lane[2], left_lane[3], h)
        right_lane = _scale_line(right_lane[0], right_lane[1], right_lane[2], right_lane[3], h)
        
        # draw the lanes
        cv2.line(img, (left_lane[0], left_lane[1]), (left_lane[2], left_lane[3]), (0, 0, 255), 5)
        cv2.line(img, (right_lane[0], right_lane[1]), (right_lane[2], right_lane[3]), (0, 0, 255), 5)

    return img

def draw_rectangle(img, rect):
    global display_on
    if display_on and len(rect) == 4:
        cv2.rectangle(img, (rect[0], rect[1]), (rect[2], rect[3]), (255, 255, 0), 1)

    return img


def draw_roi(img, rect):
    global display_on
    if display_on and len(rect) == 4:
        cv2.rectangle(img, (rect[0], rect[1]), (rect[2], rect[3]), (0, 0, 0), 3)

    return img

def draw_error(img, error, x_mid, y_mid):
    global display_on
    if display_on:
        l = 20
        x = x_mid + error

        cv2.line(img, (x-l, y_mid), (x+l, y_mid), (255, 0, 255), 2)
        cv2.line(img, (x, y_mid - l), (x, y_mid + l), (255, 0, 255), 2)
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

def draw_midline(img, x_offset, y_offset):
    global display_on
    if display_on:
        cv2.line(img, (0, y_offset ), (width, y_offset), (255, 0, 0), 1)
        cv2.line(img, (x_offset, 0 ), (x_offset, height), (255, 0, 0), 1)
    return img
