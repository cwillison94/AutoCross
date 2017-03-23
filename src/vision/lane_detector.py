from __future__ import division

import cv2
import numpy as np
import timeit


'''Defaults for Hough Line P Transform'''
HOUGH_MIN_LINE_LENGTH = 5
HOUGH_MAX_LINE_GAP = 30
HOUGH_THRESHOLD_VOTES = 10

# Region of interest theta. Only lines with angles
# greater than ROI_THETA radians will be processed
# for potential lanes.
ROI_THETA = 0.3

# Default percent of height to calculate base distance.
#     Recommended from 0.8 to 1
BASE_DISTANCE_HEIGHT_MODIFIER = 0.90


class LaneDetector:
    def __init__(self, road_horizon, width, height, base_dist_height_mod = BASE_DISTANCE_HEIGHT_MODIFIER, prob_hough= True, debug_mode = False):
        self.prob_hough = prob_hough
        self.road_horizon = road_horizon

        #frame width and height
        self.width = width
        self.height = height

        self.base_dist_height_mod = base_dist_height_mod

        self.base_distance_height = self.base_dist_height_mod * self.height

        # only look for lanes in this region
        # this 0.9 is NOT the same as the base distance height modifier
        self.lane_roi = 0.85 * self.height

        self.mid_x = self.width / 2

        self.debug_mode = debug_mode



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

    def _scale_line(self, x1, y1, x2, y2):
        # scale the farthest point of the segment to be on the drawing horizon
        if x1 == x2:
            if y1 < y2:
                y1 = self.road_horizon
                y2 = self.height
                return x1, y1, x2, y2
            else:
                y2 = self.road_horizon
                y1 = self.height
                return x1, y1, x2, y2
        if y1 < y2:
            m = (y1-y2)/(x1-x2)
            x1 = ((self.road_horizon-y1)/m) + x1
            y1 = self.road_horizon
            x2 = ((self.height-y2)/m) + x2
            y2 = self.height
        else:
            m = (y2-y1)/(x2-x1)
            x2 = ((self.road_horizon-y2)/m) + x2
            y2 = self.road_horizon
            x1 = ((self.height-y1)/m) + x1
            y1 = self.height
        return int(x1), int(y1), int(x2), int(y2)

    def detect(self, frame):
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        roiy_end = frame.shape[0]
        roix_end = frame.shape[1]
        roi_for_lane = img[self.lane_roi:roiy_end, 0:roix_end]
        blur_for_lane = cv2.medianBlur(roi_for_lane, 5)
        contours_for_lane = cv2.Canny(blur_for_lane, 50, 200)

        left_bound = None
        right_bound = None
        left_lane = None
        right_lane = None

        if self.prob_hough:
            lines = cv2.HoughLinesP(contours_for_lane, 1, np.pi/180, HOUGH_THRESHOLD_VOTES, minLineLength=HOUGH_MIN_LINE_LENGTH, maxLineGap=HOUGH_MAX_LINE_GAP)
        else:
            lines = self._standard_hough(contours_for_lane, HOUGH_THRESHOLD_VOTES)

        if lines is not None:
             # scale points from ROI coordinates to full frame coordinates
            lines = lines+np.array([0, self.lane_roi, 0, self.lane_roi]).reshape((1, 1, 4))

            for l in lines:
                # find the rightmost line of the left half of the frame and the leftmost line of the right half
                for x1, y1, x2, y2 in l:
                    #TODO: Consider convert this to calculate slope and somehow normalizing? This could be faster than arctan
                    theta = np.arctan2(y2-y1, x2-x1)
                    theta_abs = np.abs(theta)  # line angle WRT horizon
                    theta_deg = np.rad2deg(theta)
                    if theta_abs > ROI_THETA:  # ignore lines with a small angle WRT horizon
                    #if np.abs(theta_deg) > 35:
                        dist_with_modifier, dist = self._base_distance(x1, y1, x2, y2)

                        if left_bound is None and dist < 0 and x1 < self.mid_x and x2 < self.mid_x: # and (y1 > upper_bound or y2 > upper_bound): # and dist > -1* 1.2 * self.mid_x
                            left_bound = (x1, y1, x2, y2, theta)
                            left_dist = dist
                            left_dist_modifier = dist_with_modifier
                            left_theta = theta
                        elif right_bound is None and dist > 0 and x1 > self.mid_x and x2 > self.mid_x: # and (y1 > upper_bound or y2 > upper_bound): #and dist < 1.2 * self.mid_x
                            right_bound = (x1, y1, x2, y2, theta)
                            right_dist = dist
                            right_dist_modifier = dist_with_modifier
                            right_theta = theta
                        elif left_bound is not None and 0 > dist > left_dist  and x1 < self.mid_x and x2 < self.mid_x: # and (y1 > upper_bound or y2 > upper_bound): # and dist > -1* 1.2 * self.mid_x
                            left_bound = (x1, y1, x2, y2, theta)
                            left_dist = dist
                            left_dist_modifier = dist_with_modifier
                            left_theta = theta
                        elif right_bound is not None and 0 < dist < right_dist and x1 > self.mid_x and x2 > self.mid_x: # and (y1 > upper_bound or y2 > upper_bound): # and dist < 1.2 * self.mid_x
                            right_bound = (x1, y1, x2, y2, theta)
                            right_dist = dist
                            right_dist_modifier = dist_with_modifier
                            right_theta = theta

            # we only want to approximate at max 1 lane so if both are not found we don't approximate
            if left_bound is None and right_bound is None:
                return [None, None]

            if left_bound is not None:
                # scale line if debug mode for better visual representation
                if self.debug_mode:
                    #logging.debug("Left theta: ", np.rad2deg(left_theta))
                    left_bound = self._scale_line(left_bound[0], left_bound[1], left_bound[2], left_bound[3])

                left_lane = [int(left_bound[0]),int(left_bound[1]), int(left_bound[2]), int(left_bound[3]), left_dist_modifier, left_theta]
            else:
                # predicted left lane
                left_lane = []
                left_lane.append(0)
                left_lane.append(0)
                left_lane.append(0)
                left_lane.append(frame.shape[1])
                left_lane.append(-1 * frame.shape[0]/2)
                left_lane.append(np.pi/2)

            if right_bound is not None:
                if self.debug_mode:
                    #logging.debug("Right theta: ", np.rad2deg(right_theta))
                    right_bound = self._scale_line(right_bound[0], right_bound[1], right_bound[2], right_bound[3])

                right_lane = [int(right_bound[0]), int(right_bound[1]), int(right_bound[2]), int(right_bound[3]), right_dist_modifier, right_theta]
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

