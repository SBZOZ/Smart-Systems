import threading
import time
import random

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


class Robot:
    def __init__(self, threshold=15):
        self._is_blocked = False
        self.threshold = threshold
        self.sensors = {"front": 100, "left": 100, "right": 100}
        self.motion = MotionControl()
        self.current_speed = 0
        self.log = []

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
            log_entry = f"Sensors: {self.sensors} | Blocked: {self._is_blocked}"
            print(log_entry)
            self.log.append(log_entry)
            time.sleep(0.2)

    def start_detection(self):
        thread = threading.Thread(target=self.check_obstacles)
        thread.daemon = True
        thread.start()

    #speed controll 
    def adjust_speed(self):
        """Adapt speed based on proximity of obstacles."""
        min_dist = min(self.sensors.values())
        if min_dist < 20:
            target_speed = 25   #for slowing down
        elif min_dist < 40:
            target_speed = 50   #for a moderate speed
        else:
            target_speed = 100  #for a cruising speed
        self.set_speed(target_speed)

    def set_speed(self, target_speed):
        if target_speed > self.current_speed:
            self.accelerate(target_speed)
        elif target_speed < self.current_speed:
            self.decelerate(target_speed)

    def accelerate(self, target_speed, step=25, delay=0.2):
        for speed in range(self.current_speed, target_speed+1, step):
            self.motion.set_motor_speed(speed, speed)
            print(f"[ACCEL] Speed: {speed}%")
            self.current_speed = speed
            time.sleep(delay)

    def decelerate(self, target_speed=0, step=25, delay=0.2):
        for speed in range(self.current_speed, target_speed-1, -step):
            self.motion.set_motor_speed(speed, speed)
            print(f"[DECEL] Speed: {speed}%")
            self.current_speed = speed
            time.sleep(delay)

    #predictive avoidance segment
    def avoid_obstacle(self):
        print("[AVOID] Predictive avoidance triggered.")
        self.decelerate(0)

        #back up until front sensor is safe
        while self.sensors["front"] < self.threshold:
            print("[AVOID] Backing up...")
            self.motion.set_motor_speed(-50, -50)
            time.sleep(0.5)
            self.update_sensors()

        #chooses safest direction
        if self.sensors["left"] > self.sensors["right"]:
            print("[AVOID] Turning left (safer path).")
            self.motion.set_motor_speed(-50, 50)
        else:
            print("[AVOID] Turning right (safer path).")
            self.motion.set_motor_speed(50, -50)

        time.sleep(1)  #simulate turn
        self._is_blocked = False
        print("[AVOID] Path clear, resuming forward.")
        self.accelerate(100)

    #navigasjon 
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