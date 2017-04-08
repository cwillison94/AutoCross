import cv2

class PID:
    def __init__(self, K_p, K_i, K_d):
        self.K_p = K_p
        self.K_i = K_i
        self.K_d = K_d
        
        self.ticks = 0
        self.prev_ticks = 0
        self.dt = 0
        self.prev_error = 0
        self.error = 0
        self.integral = 0
        self.derivative = 0

    def update(self, error):
        self.error = error        
        self.ticks = cv2.getTickCount()
        self.dt = (self.ticks - self.prev_ticks) / cv2.getTickFrequency()

        self.integral = self.integral + self.error * self.dt
        self.derivative = (self.error - self.prev_error)/self.dt

        output = self.K_p * self.error + self.K_i * self.integral + self.K_d * self.derivative
        
        self.prev_ticks = self.ticks        
        self.prev_error = self.error

        return output
