# -*- coding: utf-8 -*-
"""
Created on Fri Dec 19 12:42:18 2025

@author: ThinkBook14
"""


import time
import threading
import random
import logging

logging.basicConfig(level=logging.INFO)


class RealtimeMonitor:
    def __init__(self):
        self.state = "idle"
        self.direction = "stop"
        self.speed = 0
        self.left_pwm = 0
        self.right_pwm = 0
        self.sensors = {"front": 100, "left": 100, "right": 100}
        self.fail_safe = False
        self.events = []
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
        if sensors is not None: self.sensors.update(sensors)
        if fail_safe is not None: self.fail_safe = fail_safe

    def push_event(self, event):
        timestamp = time.strftime("%H:%M:%S")
        self.events.append(f"{timestamp} - {event}")
        logging.info(event)

    def _monitor_loop(self):
        while self.running:
            print(
                "\n[LIVE DASHBOARD]"
                f"\n State      : {self.state:<10}"
                f"\n Direction  : {self.direction:<8}"
                f"\n Speed      : {self.speed:>3} cm/s"
                f"\n PWM        : L={self.left_pwm:<3}  R={self.right_pwm:<3}"
                f"\n Sensors    : Front={self.sensors['front']:<3}  "
                f"Left={self.sensors['left']:<3} Right={self.sensors['right']:<3}"
                f"\n FailSafe   : {'ON' if self.fail_safe else 'OFF'}"
                "\n" + "-" * 40
            )
            time.sleep(1)

class MotionControl:
    def __init__(self, lookup_table, monitor, accel=10, pwm_hz=1000):
        self.lookup_table = lookup_table
        self.current_speed = 0
        self.accel = accel
        self.monitor = monitor

        #the fake PWM objects
        class FakePWM:
            def __init__(self): pass
            def start(self, x=0): pass
            def change_duty_cycle(self, x): pass
            def stop(self): pass

        self.pwm_left = FakePWM()
        self.pwm_right = FakePWM()
        self.pwm_left.start(0)
        self.pwm_right.start(0)

    def get_pwm_for_speed(self, speed):
        for i in range(len(self.lookup_table) - 1):
            s1, pwm1 = self.lookup_table[i]
            s2, pwm2 = self.lookup_table[i + 1]
            if s1 <= speed <= s2:
                pwm = int((pwm1 + (pwm2 - pwm1) * (speed - s1) / (s2 - s1)))
                return self.lookup_table[i + 1][1]
        return self.lookup_table[-1][1]

    def accelerate_to_speed(self, target_speed):
        while self.current_speed < target_speed:
            self.current_speed += self.accel
            if self.current_speed > target_speed:
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
            if self.current_speed < target_speed:
                self.current_speed = target_speed

            pwm = self.get_pwm_for_speed(self.current_speed)
            self.set_motor_speed(pwm, pwm, direction="forward")
            self.monitor.update_status(state="warning" if target_speed > 0 else "blocked",
                                       speed=self.current_speed, left_pwm=pwm, right_pwm=pwm)

            logging.info(f"Decelerating to {self.current_speed} cm/s (PWM={pwm})")
            time.sleep(0.3)

    def set_motor_speed(self, left_pwm, right_pwm, direction="forward"):
        duty_left = min(100, max(0, (left_pwm / 255 * 100)))
        duty_right = min(100, max(0, (right_pwm / 255 * 100)))

        print(f"[MOTION] Direction: {direction}, Left PWM: {left_pwm}, Right PWM: {right_pwm}")
        self.pwm_left.change_duty_cycle(duty_left)
        self.pwm_right.change_duty_cycle(duty_right)

        self.monitor.update_status(direction=direction,
                                   left_pwm=left_pwm,
                                   right_pwm=right_pwm)

    def stop_all(self):
        try:
            self.pwm_left.change_duty_cycle(0)
            self.pwm_right.change_duty_cycle(0)
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
        self.is_blocked = False
        self.sensors = {"front": 100, "left": 100, "right": 100}
        self.running = True

    def start_obstacle_detection(self):
        thread = threading.Thread(target=self._obstacle_loop, daemon=True)
        thread.start()

    def _obstacle_loop(self):
        while self.running:

            #simulate the sensors
            self.sensors["front"] = random.randint(5, 100)
            self.sensors["left"] = random.randint(5, 100)
            self.sensors["right"] = random.randint(5, 100)
            self.monitor.update_status(sensors=self.sensors)

            #when its blocked 
            if self.sensors["front"] < self.distance_threshold:
                self.is_blocked = True
                self.monitor.update_status(state="blocked")
                self._emergency_stop()

            #warning 
            elif self.sensors["front"] < self.warning_distance:
                self.monitor.update_status(state="warning")
                logging.info(f"Warning: slowing down, obstacle at {self.sensors['front']} cm")
                self.motion_control.decelerate_to_speed(10)
                self.is_blocked = False

            else:
                self.monitor.update_status(state="cruising")
                self.is_blocked = False

            time.sleep(0.2)

    def _emergency_stop(self):
        self.monitor.push_event("Emergency stop initiated.")
        self.motion_control.decelerate_to_speed(0)

    def predictive_avoidance(self):
        self.monitor.push_event("Starting predictive avoidance...")
        pwm_back = self.motion_control.get_pwm_for_speed(10)
        timeout = time.time() + 5

        while self.sensors["front"] < self.distance_threshold and time.time() < timeout:
            self.monitor.update_status(state="avoidance", direction="backward", speed=10)

            left_pwm = pwm_back
            right_pwm = pwm_back
            self.motion_control.set_motor_speed(left_pwm, right_pwm, direction="backward")
            time.sleep(0.5)

            self.sensors['front'] = random.randint(5, 100)
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
            self.motion_control.set_motor_speed(turn_pwm, 0, direction="left")
        else:
            self.monitor.push_event("Turning right (predictive reroute)")
            self.motion_control.set_motor_speed(0, turn_pwm, direction="right")

        time.sleep(1)

        self.monitor.update_status(state="post-turn")
        self.motion_control.accelerate_to_speed(10)

        self.monitor.push_event("Resuming normal navigation")
        self.monitor.update_status(state="cruising", fail_safe=False)
        self.motion_control.accelerate_to_speed(20)


if __name__ == "__main__":

    monitor = RealtimeMonitor()
    lookup_table = [(0, 0), (10, 50), (20, 100), (30, 150)]

    motion_control = MotionControl(lookup_table, monitor, accel=10, pwm_hz=1000)
    robot = RobotSafetySystem(motion_control, monitor)
    robot.start_obstacle_detection()

    try:
        while True:
            if not robot.is_blocked:
                motion_control.accelerate_to_speed(20)
            time.sleep(1)

    except KeyboardInterrupt:
        print("System stopping...")

    finally:
        robot.running = False
        monitor.running = False
        motion_control.stop_all()
        print("System stopped cleanly.")
