import threading
import time
import random
import logging

#logging set-up 
logging.basicConfig(
    filename="robot_log.txt",
    level=logging.INFO,
    format="%(asctime)s - %(message)s"
)

class MotionControl:
    """Simulated motion control with PWM lookup and adaptive speed control."""
    def __init__(self):
        self.speed_lookup = {0: 0, 25: 64, 50: 128, 75: 192, 100: 255}

    def get_pwm_for_speed(self, speed):
        return self.speed_lookup.get(speed, 0)

    def set_motor_speed(self, left_speed, right_speed):
        left_pwm = self.get_pwm_for_speed(abs(left_speed))
        right_pwm = self.get_pwm_for_speed(abs(right_speed))
        print(f"[MOTOR] Left PWM: {left_pwm}, Right PWM: {right_pwm}")
        logging.info(f"Motor set: Left={left_pwm}, Right={right_pwm}")


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
            logging.info(f"Sensors: {self.sensors} | Blocked={self._is_blocked}")
            time.sleep(0.2)

    def start_detection(self):
        thread = threading.Thread(target=self.check_obstacles)
        thread.daemon = True
        thread.start()

    #adaptive speed controll
    def adjust_speed(self):
        min_dist = min(self.sensors.values())
        if min_dist < 20:
            target_speed = 25
        elif min_dist < 40:
            target_speed = 50
        else:
            target_speed = 100
        self.set_speed(target_speed)

    def set_speed(self, target_speed):
        if target_speed > self.current_speed:
            self.accelerate(target_speed)
        elif target_speed < self.current_speed:
            self.decelerate(target_speed)

    def accelerate(self, target_speed, step=25, delay=0.2):
        for speed in range(self.current_speed, target_speed+1, step):
            self.motion.set_motor_speed(speed, speed)
            logging.info(f"Accelerating to {speed}%")
            self.current_speed = speed
            time.sleep(delay)

    def decelerate(self, target_speed=0, step=25, delay=0.2):
        for speed in range(self.current_speed, target_speed-1, -step):
            self.motion.set_motor_speed(speed, speed)
            logging.info(f"Decelerating to {speed}%")
            self.current_speed = speed
            time.sleep(delay)

    #prediktive avaoindance with fail-safe
    def avoid_obstacle(self):
        logging.info("Avoidance maneuver triggered.")
        self.decelerate(0)

        timeout = time.time() + 5  # 5 second deadline
        while self.sensors["front"] < self.threshold and time.time() < timeout:
            logging.info("Backing up to clear obstacle...")
            self.motion.set_motor_speed(-50, -50)
            time.sleep(0.5)
            self.update_sensors()

        #fail-safe recovery if the timeout expires
        if time.time() >= timeout:
            logging.info("Fail-safe triggered: obstacle not cleared in time.")
            print("[FAIL-SAFE] Recovery maneuver: backing up longer.")
            self.motion.set_motor_speed(-50, -50)
            time.sleep(2)

        #choose safest direction
        if self.sensors["left"] > self.sensors["right"]:
            logging.info("Turning left (safer path).")
            self.motion.set_motor_speed(-50, 50)
        else:
            logging.info("Turning right (safer path).")
            self.motion.set_motor_speed(50, -50)

        time.sleep(1)  # simulate turn
        self._is_blocked = False
        logging.info("Path clear, resuming forward.")
        self.accelerate(100)

    #navigation 
    def navigate(self):
        if self._is_blocked:
            self.avoid_obstacle()
        else:
            self.adjust_speed()


#example usage
if __name__ == "__main__":
    robot = Robot()
    robot.start_detection()

    while True:
        robot.navigate()
        time.sleep(1)