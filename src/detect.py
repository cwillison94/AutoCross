from __future__ import division

import cv2
import numpy as np

'''Defaults for Hough Line P Transform'''
HOUGH_MIN_LINE_LENGTH = 10
HOUGH_MAX_LINE_GAP = 30
HOUGH_THRESHOLD_VOTES = 30

'''Region of interest theta. Only lines with angles greater than ROI_THETA radians will be processed
for potential lanes. '''
ROI_THETA = 0.3


class LaneDetector:
    def __init__(self, road_horizon, prob_hough=True):
        self.prob_hough = prob_hough
        self.road_horizon = road_horizon

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

    def base_distance(self, x1, y1, x2, y2, width, height):
        # Compute the distance from the where the line crosses the bottom x-axis to the mid
        # mid-line

        height *= 1
        if x2 == x1:
            return x1 - width/2

        m = (y2-y1)/(x2-x1)
        b = y1 - m*x1

        return (height - b)/m - width/2

    def _scale_line(self, x1, y1, x2, y2, frame_height):
        # scale the farthest point of the segment to be on the drawing horizon
        if x1 == x2:
            if y1 < y2:
                y1 = self.road_horizon
                y2 = frame_height
                return x1, y1, x2, y2
            else:
                y2 = self.road_horizon
                y1 = frame_height
                return x1, y1, x2, y2
        if y1 < y2:
            m = (y1-y2)/(x1-x2)
            x1 = ((self.road_horizon-y1)/m) + x1
            y1 = self.road_horizon
            x2 = ((frame_height-y2)/m) + x2
            y2 = frame_height
        else:
            m = (y2-y1)/(x2-x1)
            x2 = ((self.road_horizon-y2)/m) + x2
            y2 = self.road_horizon
            x1 = ((frame_height-y1)/m) + x1
            y1 = frame_height
        return int(x1), int(y1), int(x2), int(y2)

    def detect(self, frame):
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        roiy_end = frame.shape[0]
        roix_end = frame.shape[1]
        roi = img[self.road_horizon:roiy_end, 0:roix_end]
        blur = cv2.medianBlur(roi, 5)
        contours = cv2.Canny(blur, 50, 200)

        if self.prob_hough:
            lines = cv2.HoughLinesP(contours, 1, np.pi/180, HOUGH_THRESHOLD_VOTES, minLineLength=HOUGH_MIN_LINE_LENGTH, maxLineGap=HOUGH_MAX_LINE_GAP)
        else:
            lines = self.standard_hough(contours, HOUGH_THRESHOLD_VOTES)

        if lines is not None:
            # find nearest lines to center
            lines = lines+np.array([0, self.road_horizon, 0, self.road_horizon]).reshape((1, 1, 4))  # scale points from ROI coordinates to full frame coordinates
            left_bound = None
            right_bound = None
            left_lane = None
            right_lane = None
            for l in lines:
                # find the rightmost line of the left half of the frame and the leftmost line of the right half
                for x1, y1, x2, y2 in l:
                    theta = np.arctan2(y2-y1, x2-x1)
                    theta_abs = np.abs(theta)  # line angle WRT horizon

                    if theta_abs > ROI_THETA:  # ignore lines with a small angle WRT horizon
                        dist = self.base_distance(x1, y1, x2, y2, frame.shape[0], frame.shape[1])
                        mid = frame.shape[0]/2
                        upper_bound = 0.90 * frame.shape[1]
                        if left_bound is None and dist < 0 and x1 < mid and x2 < mid and (y1 > upper_bound or y2 > upper_bound): # and dist > -1* 1.2 * mid
                            left_bound = (x1, y1, x2, y2, theta)
                            left_dist = dist
                            left_theta = theta
                        elif right_bound is None and dist > 0 and x1 > mid and x2 > mid and (y1 > upper_bound or y2 > upper_bound): #and dist < 1.2 * mid
                            right_bound = (x1, y1, x2, y2, theta)
                            right_dist = dist
                            right_theta = theta
                        elif left_bound is not None and 0 > dist > left_dist  and x1 < mid and x2 < mid and (y1 > upper_bound or y2 > upper_bound): # and dist > -1* 1.2 * mid
                            left_bound = (x1, y1, x2, y2, theta)
                            left_dist = dist
                            left_theta = theta
                        elif right_bound is not None and 0 < dist < right_dist and x1 > mid and x2 > mid and (y1 > upper_bound or y2 > upper_bound): # and dist < 1.2 * mid
                            right_bound = (x1, y1, x2, y2, theta)
                            right_dist = dist
                            right_theta = theta


            if left_bound == None and right_bound == None:
                return [None, None]
            else:
                if left_bound != None:
                    left_bound = self._scale_line(left_bound[0], left_bound[1], left_bound[2], left_bound[3], frame.shape[0])
                    left_lane = [left_bound[0],left_bound[1], left_bound[2], left_bound[3], left_dist, left_theta]
                else:
                    # predicted
                    left_lane = []
                    left_lane.append(0)
                    left_lane.append(0)
                    left_lane.append(0)
                    left_lane.append(frame.shape[1])
                    left_lane.append(-1 * frame.shape[0]/2)
                    left_lane.append(np.pi/2)

                if right_bound != None:
                    right_bound = self._scale_line(right_bound[0], right_bound[1], right_bound[2], right_bound[3], frame.shape[0])
                    right_lane = [right_bound[0], right_bound[1], right_bound[2], right_bound[3], right_dist, right_theta] # deg = rad * 180 / np.pi
                else:
                    # predicted right lane
                    right_lane = []
                    right_lane.append(frame.shape[0])
                    right_lane.append(0)
                    right_lane.append(frame.shape[0])
                    right_lane.append(frame.shape[1])
                    right_lane.append(frame.shape[0]/2)
                    right_lane.append(np.pi/2)

            return [left_lane, right_lane]

