import threading
import time
import random

class MotionControl:
    """Simulated motion control to mimic Åsmund’s PWM interface."""
    def __init__(self):
        #example lookup table for PWM values
        self.speed_lookup = {0: 0, 25: 64, 50: 128, 75: 192, 100: 255}

    def get_pwm_for_speed(self, speed):
        """Return PWM value for given speed percentage."""
        return self.speed_lookup.get(speed, 0)

    def set_motor_speed(self, left_speed, right_speed):
        """Set motor speed using PWM values."""
        left_pwm = self.get_pwm_for_speed(left_speed)
        right_pwm = self.get_pwm_for_speed(right_speed)
        print(f"Left motor PWM: {left_pwm}, Right motor PWM: {right_pwm}")


class Robot:
    def __init__(self, threshold=15):
        self._is_blocked = False
        self.threshold = threshold
        self.sensors = {"front": 100, "left": 100, "right": 100}
        self.motion = MotionControl()

    def update_sensors(self):
        """Simulate sensor readings with random values."""
        self.sensors["front"] = random.randint(5, 100)
        self.sensors["left"] = random.randint(5, 100)
        self.sensors["right"] = random.randint(5, 100)

    def check_obstacles(self):
        """Continuous loop to monitor sensors and set flag."""
        while True:
            self.update_sensors()
            blocked = any(dist < self.threshold for dist in self.sensors.values())
            self._is_blocked = blocked
            print(f"Sensors: {self.sensors} | Blocked: {self._is_blocked}")
            time.sleep(0.2)

    def start_detection(self):
        """Start obstacle detection in a separate thread."""
        thread = threading.Thread(target=self.check_obstacles)
        thread.daemon = True
        thread.start()

    #movement usingt the MotionControl ---
    def move_forward(self, speed=100):
        print("Moving forward...")
        self.motion.set_motor_speed(speed, speed)

    def move_backward(self, speed=50):
        print("Backing up...")
        self.motion.set_motor_speed(-speed, -speed)

    def stop_motors(self):
        print("Stopping motors smoothly...")
        self.motion.set_motor_speed(0, 0)

    def turn_left(self, speed=50):
        print("Turning left...")
        self.motion.set_motor_speed(-speed, speed)

    def turn_right(self, speed=50):
        print("Turning right...")
        self.motion.set_motor_speed(speed, -speed)

    #avoidance part 
    def avoid_obstacle(self):
        print("Avoidance maneuver triggered.")
        self.stop_motors()
        self.move_backward()
        time.sleep(1)

        if self.sensors["left"] > self.threshold:
            self.turn_left()
        elif self.sensors["right"] > self.threshold:
            self.turn_right()
        else:
            print("No clear path detected, waiting...")

        #resumes forward IF clear
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