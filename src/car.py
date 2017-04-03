
import timeit
import Thread

class Car:

    def __init__(self):
        self.auto_running = False
        self.auto_thread = None

    def start_auto(self):
        self.auto_running = True
        self.auto_thread = threading.Thread(target = self.auto, args=())
        self.auto_thread.start()

    def stop_auto(self):
        self.auto_running = False
        self.auto_thread.join()

    def auto(self):

        while self.auto_running:

            pass