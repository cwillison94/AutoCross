from threading import Thread
import Queue
import cv2

STOP_SIGN_HAAR = "stop_sign_haar.xml"

class StopSignDetector:
    def __init__(self, cascade_file_path = STOP_SIGN_HAAR):
        self.cascade = stopSignCascade = cv2.CascadeClassifier(cascade_file_path)
        self.img_queue = Queue.Queue()
        self.stop_sign_detected = False
        self.normalized_distance = 100

    def start(self):
        # start the thread to read frames from the video stream
        self.running = True
        Thread(target=self.update, args=()).start()
        return self

    def push_img(self, img):
        self.img_queue.put(img)

    def update(self):
        # keep looping infinitely until the thread is stopped
        while self.running:
            while not self.img_queue.empty():
                img = self.img_queue.get()
                rects = self.get_stop_rects(img)

                # for now we will only look at the first
                if len(rects) > 0:
                    self.stop_sign_detected = True
                    x1, y1, x2, y2 = rects[0]
                    self.normalized_distance = int(100 * ((y1 + y2) / 2.)/img.shape[1])
                else:
                    self.stop_sign_detected = False
                    self.normalized_distance = 100

    def get_stop_info(self):
        return (self.stop_sign_detected, self.normalized_distance)

    def get_stop_rects(self, img):
        img_roi = img[0:img.shape[1], img.shape[0]/2:img.shape[0]]
        rects = self.cascade.detectMultiScale(cv2.cvtColor(img_roi, cv2.COLOR_BGR2GRAY), 1.1, 5, cv2.CASCADE_SCALE_IMAGE , (25, 25))

        if len(rects) == 0:
            return []

        rects[:, 2:] += rects[:, :2]

        # shift back to original image
        for i in range(len(rects)):
            rects[i][0] += img.shape[0]/2
            rects[i][2] += img.shape[0]/2

            #rects[i][1] += img.shape[1]/2
            #rects[i][3] += img.shape[1]/2
        return rects

    def stop(self):
        # indicate that the thread should be stopped
        self.running = False
