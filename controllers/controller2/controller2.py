"""robot_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from typing import Union
from controller import Robot, Motor, GPS, Accelerometer, Compass, Gyro, Lidar
import time


# Mon robot robospot

class Motors():

    def __init__(self, robot:Robot):
        
        # Initialisation de tous les moteurs
        # Roue avant
        self.fLwheel:Motor = robot.getDevice('fl_wheel_joint')
        self.fRwheel:Motor = robot.getDevice('fr_wheel_joint')
        # Roue arrière
        self.rLwheel:Motor = robot.getDevice('rl_wheel_joint')
        self.rRwheel:Motor = robot.getDevice('rr_wheel_joint')
        
        # Position des roue inf pour permettre une acceleration
        # Roue avant
        self.fLwheel.setPosition(float('inf'))
        self.fRwheel.setPosition(float('inf'))
        # Roue arrière
        self.rLwheel.setPosition(float('inf'))
        self.rRwheel.setPosition(float('inf'))
        self.stop()
    
    # Fonction qui va permettre d'avancer tout droit
    def runStraight(self):
        # Roue avant
        self.fLwheel.setAcceleration(10.0)
        self.fRwheel.setAcceleration(10.0)
        # Roue arrière
        self.rLwheel.setAcceleration(10.0)
        self.rRwheel.setAcceleration(10.0)
    
    # Fonction qui va permettre de reculer
    def runBack(self):
        # Roue avant
        self.fLwheel.setVelocity(-10.0)
        self.fRwheel.setVelocity(-10.0)
        # Roue arrière
        self.rLwheel.setVelocity(-10.0)
        self.rRwheel.setVelocity(-10.0)
    
    # Fonction qui va permettre de tourner à gauche
    def turnLeft(self):
        # Roue avant
        self.fLwheel.setAcceleration(5.0)
        self.fRwheel.setAcceleration(10.0)
        # Roue arrière
        self.rLwheel.setAcceleration(5.0)
        self.rRwheel.setAcceleration(10.0)

        # Fonction qui va permettre de tourner à gauche
    def shiftleft(self):
        # Roue avant
        self.fLwheel.setAcceleration(3.0)
        self.fRwheel.setAcceleration(0.0)
        # Roue arrière
        self.rLwheel.setAcceleration(3.0)
        self.rRwheel.setAcceleration(0.0)
    
    # Fonction qui va permettre de toruner à droite
    def turnRight(self):
        # Roue avant
        self.fLwheel.setAcceleration(10.0)
        self.fRwheel.setAcceleration(5.0)
        # Roue arrière
        self.rLwheel.setAcceleration(10.0)
        self.rRwheel.setAcceleration(5.0)
    
    # Fonction qui va permettre l'arret
    def stop(self):
        # Roue avant
        self.fLwheel.setAcceleration(0)
        self.fRwheel.setAcceleration(0)
        # Roue arrière
        self.rLwheel.setAcceleration(0)
        self.rRwheel.setAcceleration(0)



class monRobot(Robot):

    def __init__(self):
        super().__init__()
        self.timestep = int(self.getBasicTimeStep())
        # Initialisation de tous les capteurs du robots
        self.motors = Motors(self)
        self.accel:Accelerometer = self.getDevice('imu accelerometer')
        self.compass:Compass = self.getDevice('imu compass')
        self.gyro:Gyro = self.getDevice('imu gyro')
        self.lidar:Lidar = self.getDevice('laser')
        self.gps:GPS = self.getDevice('gps')
        # Capteur avant de distance
        self.fl_range_sensor = self.getDevice('fl_range')
        self.fr_range_sensor = self.getDevice('fr_range')
        # Capteur arrière de distance
        self.rl_range_sensor = self.getDevice('rl_range')
        self.rr_range_sensor = self.getDevice('rr_range')
        self.activation_captor()

        self.go_left = False
        self.go_right = False

        self.directionX_ok = False
        self.directionY_ok = False

    def activation_captor(self):
        self.gps.enable(self.timestep)
        self.accel.enable(self.timestep)
        self.compass.enable(self.timestep)
        self.gyro.enable(self.timestep)
        self.lidar.enable(self.timestep)
        self.fl_range_sensor.enable(self.timestep)
        self.fr_range_sensor.enable(self.timestep)
        self.rl_range_sensor.enable(self.timestep)
        self.rr_range_sensor.enable(self.timestep)
    

    def frontDetection(self,valueFRleftS,valueFRright):
        no_action = (self.go_left == False and self.go_right == False)
        print(" Gauche :"+str(self.go_left) + " Droite :"+str(self.go_right))
        # Detection des bords pour contourner
        if(no_action and valueFRleftS < 1.5 and valueFRleftS < valueFRright):
            self.go_left = True
            self.go_right = False
        elif(no_action and valueFRright < 1.5 and valueFRright < valueFRleftS):
            self.go_left = False
            self.go_right = True
        elif valueFRright >= 1.5 and valueFRright >= 1.5:
            self.go_left = False
            self.go_right = False
    
    def drivingRobot(self,X,Y,compass):
        if self.go_left:
            self.motors.turnLeft()
        elif self.go_right: 
            self.motors.turnRight() 
        else:
            self.runStraightToPoint(X,Y,compass)
            print(" X :"+str(X) + "Y :"+str(Y) )
        
                                #X                #Y                 #Z
    # Point GPS à ralier [-45.003356381052015, 52.28051606042675, 0.1689372910961459]
    def runStraightToPoint(self,X,Y,compass):
        pointToRallyX = -45
        pointToRallyY = 52
        directionAngleSide = -5.9
        directionAngleEnd = 6
        
        if(not self.directionX_ok and compass != directionAngleSide):
            self.motors.shiftleft()
        elif(not self.directionX_ok and compass == directionAngleSide):
            self.directionX_ok = True
        elif((self.directionX_ok and int(X) <= pointToRallyX) or (self.directionY_ok and int(Y) <= pointToRallyY)) :
            self.motors.runStraight()
        elif (not self.directionY_ok and int(X) == pointToRallyX and compass != directionAngleEnd) :
            self.motors.shiftleft()
        elif (not self.directionY_ok and int(X) == pointToRallyX and compass == directionAngleEnd) :
            self.directionY_ok = True
        # elif(X == pointToRallyX and Y == pointToRallyY):
        #     self.motors.runStraight()    


# create the Robot instance.
robot = monRobot()

# get the time step of the current world.


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(robot.timestep) != -1:
    robot.motors.stop()
    
    valueGps = robot.gps.getValues()
    valueSen = robot.fl_range_sensor.getValue()
    valueSe2 = robot.fr_range_sensor.getValue()
    values3 = robot.compass.getValues()
    print("valueFrontleft is: ", valueSen)
    print("valueFrontRight is: ", valueSe2)
    print("valueGps value is: ", valueGps)
    print("Compas is "+str(values3[2]))

    robot.frontDetection(valueSen, valueSe2)
    robot.drivingRobot(valueGps[0], valueGps[1],values3[2])
    