
import threading
import time
import random
import logging
from collections import deque
import RPi.GPIO as GPIO
from rpi_hardware_pwm import HardwarePWM

logging.basicConfig(filename="robot_log.txt", level=logging.INFO,
                    format="%(asctime)s - %(message)s")

ENBL = 12
DIRL = 20
ENBR = 21
DIRR = 26
SLEEP = 8
RESET = 7
DECAY = 16
FAULT = 25

TRIG1 = 27
ECHO1 = 17

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

for pin in [ENBL, DIRL, ENBR, DIRR, SLEEP, RESET, DECAY]:
    GPIO.setup(pin, GPIO.OUT)
GPIO.setup(FAULT, GPIO.IN)

GPIO.setup(TRIG1, GPIO.OUT)
GPIO.setup(ECHO1, GPIO.IN)

GPIO.output(ENBL, GPIO.LOW)
GPIO.output(ENBR, GPIO.LOW)
GPIO.output(DIRL, GPIO.LOW)
GPIO.output(DIRR, GPIO.LOW)
GPIO.output(SLEEP, GPIO.HIGH)
GPIO.output(RESET, GPIO.HIGH)
GPIO.output(DECAY, GPIO.LOW)

def pulse_reset(duration=0.05):
    GPIO.output(RESET, GPIO.LOW)
    time.sleep(duration)
    GPIO.output(RESET, GPIO.HIGH)

def read_fault():
    return GPIO.input(FAULT) == GPIO.HIGH

def measure_distance(trigger_pin, echo_pin, timeout=0.03):
    GPIO.output(trigger_pin, False)
    time.sleep(0.0002)
    GPIO.output(trigger_pin, True)
    time.sleep(0.00001)
    GPIO.output(trigger_pin, False)
    start_time= time.time()
    while GPIO.input(echo_pin) == 0:
        if time.time() - start_time >timeout:
            return None
    pulse_start = time.time()
    while GPIO.input(echo_pin) == 1:
        if time.time() - pulse_start > timeout:
            return None
    pulse_end = time.time()
    duration= pulse_end -pulse_start
    distance_cm = duration * 17150
    return max(0, min(400, int(distance_cm)))

class RealtimeMonitor:
    def __init__(self):
        self.state = "idle"
        self.direction = "forward"
        self.left_pwm =0
        self.right_pwm = 0
        self.speed = 0
        self.sensors = {"front":100}
        self.fail_safe = False
        self.events = deque(maxlen=10)
        self.running = True
        thread = threading.Thread(target=self._monitor_loop, daemon=True)
        thread.start()

    def update_status(self, state=None, direction=None, left_pwm=None, right_pwm=None,
                      speed=None, sensors=None, fail_safe=None):
        if state: self.state = state
        if direction: self.direction = direction
        if left_pwm is not None: self.left_pwm = left_pwm
        if right_pwm is not None: self.right_pwm = right_pwm
        if speed is not None: self.speed = speed
        if sensors: self.sensors.update(sensors)
        if fail_safe is not None: self.fail_safe = fail_safe

    def push_event(self, event):
        timestamp = time.strftime("%H:%M:%S")
        self.events.append(f"{timestamp} - {event}")
        logging.info(event)

    def _monitor_loop(self):
        while self.running:
            print(
                "\n[LIVE DASHBOARD]"
                f"\n State     : {self.state:<10}"
                f"\n Direction : {self.direction:<8}"
                f"\n Speed     : {self.speed:>3} cm/s"
                f"\n PWM       : L={self.left_pwm:<3} / R={self.right_pwm:<3}"
                f"\n Sensors   : Front={self.sensors['front']:<3}"
                f"\n FailSafe  : {'ON' if self.fail_safe else 'OFF'}"
                "\n"+"-"*40
            )
            time.sleep(1)

class MotionControl:
    def __init__(self, lookup_table, monitor, accel=10, pwm_hz=1000):
        self.lookup_table = lookup_table
        self.current_speed = 0
        self.accel = accel
        self.monitor = monitor
        self.pwm_left = HardwarePWM(pwm_channel=0, hz=pwm_hz)
        self.pwm_right = HardwarePWM(pwm_channel=1, hz=pwm_hz)
        self.pwm_left.start(0)
        self.pwm_right.start(0)

    def get_pwm_for_speed(self, speed):
        for i in range(len(self.lookup_table)- 1):
            s1, pwm1 = self.lookup_table[i]
            s2, pwm2 = self.lookup_table[i+ 1]
            if speed <= s2:
                return int(pwm1 + (pwm2 - pwm1)* (speed- s1)/(s2-s1))
        return self.lookup_table[-1][1]

    def accelerate_to_speed(self, target_speed):
        while self.current_speed <target_speed:
            self.current_speed += self.accel
            if self.current_speed >target_speed:
                self.current_speed = target_speed
            pwm = self.get_pwm_for_speed(self.current_speed)
            self.set_motor_speed(pwm, pwm, direction="forward")
            self.monitor.update_status(state="cruising", speed=self.current_speed,
                                       left_pwm=pwm, right_pwm=pwm)
            logging.info(f"Accelerating to {self.current_speed} cm/s (PWM={pwm})")
            time.sleep(0.3)

    def decelerate_to_speed(self, target_speed):
        while self.current_speed > target_speed:
            self.current_speed -= self.accel
            if self.current_speed <target_speed:
                self.current_speed = target_speed
            pwm = self.get_pwm_for_speed(self.current_speed)
            self.set_motor_speed(pwm, pwm, direction="forward")
            self.monitor.update_status(state="warning" if target_speed > 0 else "blocked",
                                       speed=self.current_speed, left_pwm=pwm, right_pwm=pwm)
            logging.info(f"Decelerating to {self.current_speed} cm/s (PWM={pwm})")
            time.sleep(0.3)

    def _set_direction_enable(self, direction):
        GPIO.output(SLEEP, GPIO.HIGH)
        if direction == "forward":
            GPIO.output(DIRL, GPIO.HIGH)
            GPIO.output(DIRR, GPIO.HIGH)
        elif direction == "backward":
            GPIO.output(DIRL, GPIO.LOW)
            GPIO.output(DIRR, GPIO.LOW)
        GPIO.output(ENBL, GPIO.HIGH)
        GPIO.output(ENBR, GPIO.HIGH)

    def set_motor_speed(self, left_pwm, right_pwm, direction="forward"):
        if read_fault():
            logging.info("Controller fault detected. Resetting...")
            pulse_reset()
        self._set_direction_enable(direction)
        duty_left = min(100, max(0, left_pwm/255*100))
        duty_right = min(100, max(0, right_pwm/255*100))
        self.pwm_left.change_duty_cycle(duty_left)
        self.pwm_right.change_duty_cycle(duty_right)
        print(f"[MOTION] Direction: {direction}, Left PWM: {left_pwm}, Right PWM: {right_pwm}")
        logging.info(f"Motor command: {direction}, Left PWM={left_pwm}, Right PWM={right_pwm}")
        self.monitor.update_status(direction=direction, left_pwm=left_pwm, right_pwm=right_pwm)

    def stop_all(self):
        try:
            self.pwm_left.change_duty_cycle(0)
            self.pwm_right.change_duty_cycle(0)
            GPIO.output(ENBL, GPIO.LOW)
            GPIO.output(ENBR, GPIO.LOW)
            self.pwm_left.stop()
            self.pwm_right.stop()
        except Exception as e:
            logging.info(f"PWM stop error: {e}")

class RobotSafetySystem:
    def __init__(self, motion_control, monitor, distance_threshold=20, warning_distance=40):
        self.motion_control = motion_control
        self.monitor = monitor
        self.distance_threshold = distance_threshold
        self.warning_distance = warning_distance
        self._is_blocked = False
        self.sensors= {"front": 100}
        self.running= True

    def start_obstacle_detection(self):
        thread = threading.Thread(target=self._obstacle_loop, daemon=True)
        thread.start()

    def _read_front_distance(self):
        d = measure_distance(TRIG1, ECHO1)
        return d if d is not None else random.randint(5, 100)

    def _obstacle_loop(self):
        while self.running:
            #reads the sensors (front via ultrasonic, sides simulated are)
            self.sensors["front"] = self._read_front_distance()
            self.sensors["left"], self.sensors["right"] = self._read_side_distances()
            self.monitor.update_status(sensors=self.sensors)

            if self.sensors["front"] < self.distance_threshold:
                self._is_blocked = True
                self.monitor.update_status(state="blocked")
                self.monitor.push_event(f"Obstacle detected! Front={self.sensors['front']} cm")
                self._emergency_stop()
                self._predictive_avoidance()
            elif self.sensors["front"] < self.warning_distance:
                self.monitor.update_status(state="warning")
                logging.info(f"Warning: slowing down, obstacle ahead at {self.sensors['front']} cm")
                self.motion_control.decelerate_to_speed(10)
                self._is_blocked = False
            else:
                self.monitor.update_status(state="cruising")
                self._is_blocked = False
            time.sleep(0.2)

    def _emergency_stop(self):
        self.monitor.push_event("Emergency stop initiated.")
        self.motion_control.decelerate_to_speed(0)

    def _predictive_avoidance(self):
        self.monitor.push_event("Starting predictive avoidance...")
        pwm_back = self.motion_control.get_pwm_for_speed(10)
        timeout = time.time() + 5
        while self.sensors["front"] < self.distance_threshold and time.time() < timeout:
            self.monitor.update_status(state="avoidance", direction="backward", speed=10,
                                       left_pwm=pwm_back, right_pwm=pwm_back)
            self.motion_control.set_motor_speed(pwm_back, pwm_back, direction="backward")
            time.sleep(0.5)
            self.sensors["front"] = self._read_front_distance()
            self.monitor.update_status(sensors=self.sensors)

        if time.time() >= timeout:
            self.monitor.push_event("Fail-safe triggered")
            self.monitor.update_status(state="recovery", fail_safe=True)
            time.sleep(2)
        else:
            self.monitor.update_status(fail_safe=False)

        turn_pwm = self.motion_control.get_pwm_for_speed(8)
        if self.sensors["left"] > self.sensors["right"]:
            self.monitor.push_event("Turning left (predictive reroute)")
            self.monitor.update_status(state="turning")
            #left turn: slow/stall left side, run right
            self.motion_control.set_motor_speed(0, turn_pwm, direction="left")
        else:
            self.monitor.push_event("Turning right (predictive reroute)")
            self.monitor.update_status(state="turning")
            #right turn: run left, slow/stall right side
            self.motion_control.set_motor_speed(turn_pwm, 0, direction="right")
        time.sleep(1)

        #move forward after turn
        self.monitor.update_status(state="post-turn")
        self.motion_control.accelerate_to_speed(10)
        time.sleep(1)

        #resume cruising
        self.monitor.push_event("Resuming normal navigation")
        self.monitor.update_status(state="cruising", fail_safe=False)
        self.motion_control.accelerate_to_speed(20)

#main 
if __name__ == "main":
    try:
        monitor =RealtimeMonitor()
        lookup_table = [(0, 0), (10, 50), (20, 100), (30, 150)]
        motion_control = MotionControl(lookup_table, monitor, accel=10, pwm_hz=1000)
        robot= RobotSafetySystem(motion_control, monitor)
        robot.start_obstacle_detection()

        while True:
            if not robot._is_blocked:
                motion_control.accelerate_to_speed(20)
            time.sleep(1)
    except KeyboardInterrupt:
        print("System stopping...")
    finally:
        #for shutdown 
        robot.running = False
        monitor.running = False
        motion_control.stop_all()
        GPIO.cleanup()