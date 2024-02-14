from controller import Robot, Motor, GPS
import sys
import math


import numpy as np


def clamp(value, value_min, value_max):
    return min(max (value, value_min), value_max)

class Mavic(Robot):
    K_VERTICAL_THRUST = 68.5
    K_VERTICAL_OFFSET = 0.6
    K_VERTICAL_P = 3.0
    K_ROLL_P = 50.0
    K_PITCH_P = 30.0
    MAX_YAW_DISTURBANCE = 0.4
    MAX_PITCH_DISTURBANCE = 1.0
    target_precision = 1.0  

    def __init__(self):
        Robot.__init__(self)
        self.time_step = int(self.getBasicTimeStep())
        self.init_devices()
        self.waypoints = [
            (-45, 56.0, 0.06), (-50.8, 72.8, 3.9), (-57.2, 80.07, 3.9), (-60.5, 72.2, 6.53),
            (-60.58, 65.05, 6.53), (-64.29, 59.59, 10.46), (-72.51, 59.6, 9.9), (-74.19, 67.8, 1.72),
            (-74.1, 87.55, 1.72)
        ]
        self.target_index = 0

    def init_devices(self):
         self.camera = self.getDevice("camera") 
         self.camera.enable(self.time_step) 
         # Inertial  Unit 
         self.imu = self.getDevice("inertial unit") 
         self.imu.enable(self.time_step) # GPS 
         self.gps = self.getDevice("gps") 
         self.gps.enable(self.time_step) 
         # Gyroscope 
         self.gyro = self.getDevice("gyro") 
         self.gyro.enable(self.time_step) 
         # Motors 
         self.motors = [] 
         motor_names = ["front left propeller", "front right propeller", "rear left propeller", "rear right propeller"] 
         for name in motor_names: 
             motor = self.getDevice(name) 
             motor.setPosition(float('inf')) 
             # Set to velocity control mode 
             motor.setVelocity(0) 
             # Initially stopped 
             self.motors.append(motor) 

    def adjust_motors_to_navigate(self, current_position, target_position, roll, pitch, yaw):
        dx = target_position[0] - current_position[0]
        dy = target_position[1] - current_position[1]
        dz = target_position[2] - current_position[2]
        distance_xy = math.sqrt(dx**2 + dy**2)
        target_bearing = math.atan2(dy, dx)
        yaw_error = target_bearing - yaw
        yaw_disturbance = clamp(yaw_error, -self.MAX_YAW_DISTURBANCE, self.MAX_YAW_DISTURBANCE)
        pitch_disturbance = clamp(-distance_xy / 10.0, -self.MAX_PITCH_DISTURBANCE, self.MAX_PITCH_DISTURBANCE)
        vertical_input = self.K_VERTICAL_P * (dz + self.K_VERTICAL_OFFSET)
        vertical_input = clamp(vertical_input + self.K_VERTICAL_THRUST, 0, self.K_VERTICAL_THRUST * 2)

    def run(self):
        while self.step(self.time_step) != -1:
            roll, pitch, yaw = self.imu.getRollPitchYaw()
            x, y, altitude = self.gps.getValues()
            current_position = [x, y, altitude]
            if self.target_index < len(self.waypoints):
                target_position = self.waypoints[self.target_index]
                distance = np.linalg.norm(np.array(target_position) - np.array(current_position))
                if distance < self.target_precision:
                    print(f"Waypoint {self.target_index} reached.")
                    self.target_index += 1
                    if self.target_index >= len(self.waypoints):
                        print("Final waypoint reached. Stopping.")
                        for motor in self.motors:
                            motor.setVelocity(0)
                        break
                else:
                    self.adjust_motors_to_navigate(current_position, target_position, roll, pitch, yaw)

if __name__ == "__main__": 
mavic_drone = Mavic() 
mavic_drone.run()


