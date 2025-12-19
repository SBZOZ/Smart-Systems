import threading
import time
import random

class MotionControl:
    """Simulated motion control to mimic PWM interface with acceleration/deceleration."""
    def __init__(self):
        #example lookup table for PWM values
        self.speed_lookup = {0: 0, 25: 64, 50: 128, 75: 192, 100: 255}

    def get_pwm_for_speed(self, speed):
        """Return PWM value for given speed percentage."""
        return self.speed_lookup.get(speed, 0)

    def set_motor_speed(self, left_speed, right_speed):
        """Set motor speed using PWM values."""
        left_pwm = self.get_pwm_for_speed(abs(left_speed))
        right_pwm = self.get_pwm_for_speed(abs(right_speed))
        print(f"Left motor PWM: {left_pwm}, Right motor PWM: {right_pwm}")


class Robot:
    def __init__(self, threshold=15):
        self._is_blocked = False
        self.threshold = threshold
        self.sensors = {"front": 100, "left": 100, "right": 100}
        self.motion = MotionControl()
        self.current_speed = 0

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

    #acceleration adn deacceleration  
    def accelerate(self, target_speed, step=25, delay=0.2):
        for speed in range(self.current_speed, target_speed+1, step):
            self.motion.set_motor_speed(speed, speed)
            print(f"Accelerating: {speed}%")
            self.current_speed = speed
            time.sleep(delay)

    def decelerate(self, step=25, delay=0.2):
        for speed in range(self.current_speed, -1, -step):
            self.motion.set_motor_speed(speed, speed)
            print(f"Decelerating: {speed}%")
            self.current_speed = speed
            time.sleep(delay)

    #movement segment  
    def move_forward(self, target_speed=100):
        print("Resuming forward with acceleration...")
        self.accelerate(target_speed)

    def move_backward(self, target_speed=50):
        print("Backing up...")
        self.motion.set_motor_speed(-target_speed, -target_speed)

    def stop_motors(self):
        print("Emergency stop with deceleration...")
        self.decelerate()

    def turn_left(self, speed=50):
        print("Turning left...")
        self.motion.set_motor_speed(-speed, speed)

    def turn_right(self, speed=50):
        print("Turning right...")
        self.motion.set_motor_speed(speed, -speed)

    #the dynamic Avoidance
    def avoid_obstacle(self):
        print("Avoidance maneuver triggered.")
        self.stop_motors()
        self.move_backward()
        time.sleep(1)

        #the decision part
        if self.sensors["left"] > self.threshold:
            self.turn_left()
        elif self.sensors["right"] > self.threshold:
            self.turn_right()
        else:
            print("No clear path detected, waiting...")

        #resume forward with acceleration if clear
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

