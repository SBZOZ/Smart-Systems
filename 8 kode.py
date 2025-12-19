import threading
import time

class ObstacleDetection:
    def __init__(self, threshold=15):
        self.robot_is_blocked = False
        self.threshold = threshold
        self.sensors = {"front": 30, "left": 40, "right": 35}  # simulated values

    def check_obstacles(self):
        while True:
            for direction, dist in self.sensors.items():
                print(f"{direction} sensor: {dist} cm")
                if dist < self.threshold:
                    self.robot_is_blocked = True
                    print("Obstacle detected! Flag set.")
                    break
            time.sleep(0.2)  #5 checks per second

    def start(self):
        thread = threading.Thread(target=self.check_obstacles)
        thread.daemon = True
        thread.start()

#example usage
detector = ObstacleDetection()
detector.start()

while True:
    if detector.robot_is_blocked:
        print("Navigation paused due to obstacle.")
        time.sleep(2)
        detector.robot_is_blocked = False
        print("Resuming navigation...")
    else:
        print("Path clear, moving forward.")
    time.sleep(1)