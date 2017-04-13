from __future__ import division

import cv2
import numpy as np
import time
import math

'''Defaults for Hough Line P Transform'''
#HOUGH_MIN_LINE_LENGTH = 5
#HOUGH_MAX_LINE_GAP = 30

HOUGH_MIN_LINE_LENGTH = 50
HOUGH_MAX_LINE_GAP = 30

HOUGH_THRESHOLD_VOTES = 10

# Region of interest theta. Only lines with angles
# greater than ROI_THETA radians will be processed
# for potential lanes.
ROI_THETA = 0.3

# Default percent of height to calculate base distance.
#     Recommended from 0.8 to 1
BASE_DISTANCE_HEIGHT_MODIFIER = 0.75

LANE_WIDTH_PX = 360


class LaneDetector:
    def __init__(self, width, height, enable_stop_line_detection = False, debug_mode = False):
        
        self.prob_hough = True

        #frame width and height
        self.width = width
        self.height = height

        self.base_distance_height = BASE_DISTANCE_HEIGHT_MODIFIER * height

        # only look for lanes in this region
        #x1 y1 x2 y2
        self.roi = [0, 0.7 * self.height ,self.width, 0.85*self.height ]
     
        self.approx_base_dist = int(1 * self.width / 2)

        self.prev_left_lane = None
        self.prev_right_lane = None

        self.mid_x = self.width / 2

        self.debug_mode = debug_mode
        self.enable_stop_line_detection = enable_stop_line_detection

        self.approximated_right_bound = [width, 0, width, height, self.approx_base_dist, True]
        self.approximated_left_bound = [0, 0, 0, height, -1 * self.approx_base_dist, True]


    def _standard_hough(self, img, init_vote):
        # Hough transform wrapper to return a list of points like PHough does
        lines = cv2.HoughLines(img, 1, np.pi/180, init_vote)
        points = [[]]
        for l in lines:
            for rho, theta in l:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a*rho
                y0 = b*rho
                x1 = int(x0 + 1000*(-b))
                y1 = int(y0 + 1000*a)
                x2 = int(x0 - 1000*(-b))
                y2 = int(y0 - 1000*a)
                points[0].append((x1, y1, x2, y2))
        return points

    def _base_distance(self, x1, y1, x2, y2):
        # Compute the distance from the where the line crosses the bottom x-axis to the mid-line

        if x2 == x1:
            return x1 - self.mid_x, x1 - self.mid_x

        m = (y2-y1)/(x2-x1)
        b = y1 - m*x1

        # (base dist with height with modifier, base dist without height modifier)
        # height mod pushes the calculated base distance forward (looking into the "future")
        # |
        # |         |
        # |         /         /
        # |        /|        /
        # |       / |       /
        # |      /  |      /
        # | - - /- -|- - -/ -  <- Where base distance with height moifier is calculated
        # |____/____|____/____ Bottom of image (front of car) - where base distance is calculate
        base_dist_mod = (self.base_distance_height  - b)/m - self.mid_x
        base_dist = (self.height  - b)/m - self.mid_x

        return base_dist_mod, base_dist
        
    def line_length(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2) ** 2 + (y1-y2) ** 2)

    def _get_line_intersection(self, line1, line2):

        if line1 is None or line2 is None:
            return (None, None)
        l1_x1, l1_y1, l1_x2, l1_y2 = line1  
        l2_x1, l2_y1, l2_x2, l2_y2 = line2 

        # Edge cases
        if l1_x2 == l1_x1 and l2_x2 == l2_x1:
            return (None, None)
        elif l1_x2 == l1_x1:
            m_1 = (l2_y2 - l2_y1) / (l2_x2 - l2_x1)
            b_1 = l2_y1 - m_1 * l2_x1
            x_int = l1_x2
            y_int = m_1 * x_int + b_1
            return (x_int, y_int)
        elif l2_x2 == l2_x1:
            m_0 = (l1_y2 - l1_y1) / (l1_x2 - l1_x1)
            b_0 = l1_y1 - m_0 * l1_x1
            x_int = l2_x1
            y_int = m_0 * x_int + b_0
            return (x_int, y_int)
        else:
            m_0 = (l1_y2 - l1_y1) / (l1_x2 - l1_x1)
            b_0 = l1_y1 - m_0 * l1_x1

            m_1 = (l2_y2 - l2_y1) / (l2_x2 - l2_x1)
            b_1 = l2_y1 - m_1 * l2_x1

            if m_0 == m_1:
                return (None, None)

            x_int = (b_1 - b_0) / (m_0 - m_1)

            y_int = m_0 * x_int + b_0

            return (x_int, y_int)

    def _between(self, value, lower, upper):
        return value >= lower and value <= upper

    def detect(self, frame):
        lane_detect_start = time.time()        
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        roi_for_lane = img[self.roi[1]:self.roi[3], self.roi[0]:self.roi[2]]
        blur_for_lane = cv2.medianBlur(roi_for_lane, 5)
        contours_for_lane = cv2.Canny(blur_for_lane, 100, 200)

        left_bound = None
        right_bound = None
        left_lane = None
        right_lane = None
        left_dist_modifier = None
        right_dist_modifier = None

        stop_line = None
        stop_line_angle = None

        potential_stop_lines = []

        if self.prob_hough:
            lines = cv2.HoughLinesP(contours_for_lane, 1, np.pi/180, HOUGH_THRESHOLD_VOTES, minLineLength=HOUGH_MIN_LINE_LENGTH, maxLineGap=HOUGH_MAX_LINE_GAP)
        else:
            lines = self._standard_hough(contours_for_lane, HOUGH_THRESHOLD_VOTES)

        if lines is not None:
            # find the rightmost line of the left half of the frame and the leftmost line of the right half
            for l in lines:

                # get line coordinates scaled to full frame
                x1, y1, x2, y2 = l[0]
                y2 = y2 + self.roi[1]
                y1 = y1 + self.roi[1]

                theta = np.arctan2(y2-y1, x2-x1)
                theta_abs = np.abs(theta)  # line angle WRT horizon
                theta_deg = np.rad2deg(theta)
                theta_deg_abs = np.abs(theta_deg)
                
                # ignore lines with a small angle WRT horizon
                if theta_deg_abs > 25:
                    dist_with_modifier, dist = self._base_distance(x1, y1, x2, y2)

                    #possible left lane boundary
                    if (left_bound == None or 0 > dist > left_dist)  and x1 < self.mid_x and x2 < self.mid_x: 
                        left_bound = (x1, y1, x2, y2)
                        left_dist = dist
                        left_dist_modifier = dist_with_modifier

                    #possible right lane boundary
                    elif (right_bound == None or 0 < dist < right_dist) and x1 > self.mid_x and x2 > self.mid_x:
                        right_bound = (x1, y1, x2, y2)
                        right_dist = dist
                        right_dist_modifier = dist_with_modifier
                
                #elif theta_abs < 30:    
                else:
                    if self.enable_stop_line_detection:
                        # Potential stop lines
                        if x2 != x1:
                            m = (y2 - y1) / float((x2 - x1))
                            length = self.line_length(x1, y1, x2, y2) 
                            # print "m = " + str(m)
                            if length > 30:
                                potential_stop_lines.append([x1, y1, x2, y2, theta_deg_abs, length])                         

            # we only want to approximate at max 1 lane so if both are not found we don't approximate
            if left_bound is None and right_bound is None:
                print('no lanes found')
                self.prev_left_lane = None
                self.prev_right_lane = None
                return [None, None, None]
            elif self.enable_stop_line_detection:
                # Check for stop line
                for x1, y1, x2, y2, theta_abs, length in potential_stop_lines:
                    x1_int, y1_int = self._get_line_intersection(left_bound, (x1, y1, x2, y2))
                    x2_int, y2_int = self._get_line_intersection(right_bound, (x1, y1, x2, y2))

                    # print "LEFT: x_int: " + str(x1_int) + " y_int: " + str(y1_int) 
                    # print "RIGHT: x_int: " + str(x2_int) + " y_int: " + str(y2_int) 

                    # If the intersects are within the frame region
                    if (x1_int is not None and y1_int is not None and x2_int is not None and y2_int is not None 
                        and self._between(x1_int - self.mid_x, left_dist_modifier, right_dist_modifier) and self._between(x2_int - self.mid_x, left_dist_modifier, right_dist_modifier) 
                        and self._between(y1_int, 0,300) and self._between(y1_int, 0, 300)):
                        # print "Stop Line Found angle: " + str(theta_abs) + " len=" + str(length)
                        stop_line = [int(x1), int(y1), int(x2), int(y2)]

            # determine if non-approximated lane is left or right
            if left_bound is None or right_bound is None:
                if left_bound is not None:
                    line = [int(left_bound[0]),int(left_bound[1]), int(left_bound[2]), int(left_bound[3])]
                else:
                    line = [int(right_bound[0]), int(right_bound[1]), int(right_bound[2]), int(right_bound[3])]

                x1 = line[0]
                y1 = line[1]
                x2 = line[2]
                y2 = line[3]

                dist_mod, _ = self._base_distance(x1, y1, x2, y2) 

                theta = np.abs(np.rad2deg(np.arctan2(y2-y1, x2-x1)))
                print('theta of unapproximated lane: ', theta)

                # left lane
                if theta < 90:
                    print('detected lane should be left')
                    left_lane = [int(x1),int(y1), int(x2), int(y2), dist_mod, False]
                    right_lane = self.approximated_right_bound
                    print('right bound approximated')

                # right lane
                else:
                    print('detected lane should be right')
                    right_lane = [int(x1),int(y1), int(x2), int(y2), dist_mod, False]
                    left_lane = self.approximated_left_bound
                    print('left bound approximated')


            else:

                left_lane = [int(left_bound[0]),int(left_bound[1]), int(left_bound[2]), int(left_bound[3]), left_dist_modifier, False]
                right_lane = [int(right_bound[0]), int(right_bound[1]), int(right_bound[2]), int(right_bound[3]), right_dist_modifier, False]
          
        self.prev_left_lane = left_lane
        self.prev_right_lane = right_lane
        return [left_lane, right_lane, stop_line]

