import threading
import time
import random

class Robot:
    def __init__(self, threshold=15):
        #the shared flag for navigation integration
        self._is_blocked = False
        self.threshold = threshold
        #simulated sensor readings (front, left, right)
        self.sensors = {"front": 100, "left": 100, "right": 100}
        #the urrent speed (simulated)
        self.speed = 100

    def update_sensors(self):
        """Simulate sensor readings with random values."""
        self.sensors["front"] = random.randint(5, 100)
        self.sensors["left"] = random.randint(5, 100)
        self.sensors["right"] = random.randint(5, 100)

    def check_obstacles(self):
        """Continuous loop to monitor sensors and set flag."""
        while True:
            self.update_sensors()
            blocked = False
            for direction, dist in self.sensors.items():
                print(f"{direction} sensor: {dist} cm")
                if dist < self.threshold:
                    blocked = True
            self._is_blocked = blocked
            if blocked:
                print("Obstacle detected! Flag set.")
            else:
                print("Path clear.")
            time.sleep(0.2)  #again, 5 checks per second

    def start_detection(self):
        """Start obstacle detection in a separate thread."""
        thread = threading.Thread(target=self.check_obstacles)
        thread.daemon = True
        thread.start()

    #motor control (simulated)
    def stop_motors(self):
        self.speed = 0
        print("Motors stopped (emergency stop).")

    def move_forward(self):
        self.speed = 100
        print("Moving forward at speed 100.")

    def move_backward(self):
        self.speed = 50
        print("Backing up at speed 50.")

    def turn_left(self):
        print("Turning left to avoid obstacle.")

    def turn_right(self):
        print("Turning right to avoid obstacle.")

    #obstacle avoidance 
    def avoid_obstacle(self):
        print("Avoidance maneuver triggered.")
        self.stop_motors()
        self.move_backward()
        time.sleep(1)

        #decide which side is clear
        if self.sensors["left"] > self.threshold:
            self.turn_left()
        elif self.sensors["right"] > self.threshold:
            self.turn_right()
        else:
            print("No clear path detected, waiting...")

        #final check before resuming, for safety 
        if all(dist > self.threshold for dist in self.sensors.values()):
            self._is_blocked = False
            print("Path clear, resuming forward movement.")
            self.move_forward()

#example usage
if __name__ == "__main__":
    robot = Robot()
    robot.start_detection()

    while True:
        if robot._is_blocked:
            robot.avoid_obstacle()
        else:
            robot.move_forward()
        time.sleep(1)