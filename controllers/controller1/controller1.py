"""robot_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from typing import Union
from controller import Robot, Motor, GPS, Accelerometer, Compass, Gyro, Lidar, DistanceSensor
import time


# Mon robot robospot

class Motors():

    def __init__(self, robot:Robot):
        
        # Initialisation de tous les moteurs
        # Roue avant
        self.fLwheel:Motor = robot.getDevice('front_left_wheel_joint')
        self.fRwheel:Motor = robot.getDevice('front_right_wheel_joint')
        # Roue arrière
        self.rLwheel:Motor = robot.getDevice('back_left_wheel_joint')
        self.rRwheel:Motor = robot.getDevice('back_right_wheel_joint')
        
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
    
    # Fonction qui va permettre de toruner à droite
    def turnRight(self):
        # Roue avant
        self.fLwheel.setAcceleration(10.0)
        self.fRwheel.setAcceleration(6.0)
        # Roue arrière
        self.rLwheel.setAcceleration(10.0)
        self.rRwheel.setAcceleration(6.0)
    
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

        self.gps:GPS = self.getDevice('gps')
        
        self.capteur1:DistanceSensor = self.getDevice('distance sensor')

        self.capteur2:DistanceSensor = self.getDevice('distance sensor')

        """""
        self.gyro:Gyro = self.getDevice('imu gyro')
        self.lidar:Lidar = self.getDevice('laser')
        # Capteur avant de distance
        self.fl_range_sensor = self.getDevice('fl_range')
        self.fr_range_sensor = self.getDevice('fr_range')
        # Capteur arrière de distance
        self.rl_range_sensor = self.getDevice('rl_range')
        self.rr_range_sensor = self.getDevice('rr_range')
        """
        self.activation_captor()

        self.go_left = False
        self.go_right = False

    def activation_captor(self):
        
        self.gps.enable(self.timestep)
        
        self.capteur1.enable(self.timestep)
        self.capteur2.enable(self.timestep)

        """""
        self.gyro.enable(self.timestep)
        self.lidar.enable(self.timestep)
        self.fl_range_sensor.enable(self.timestep)
        self.fr_range_sensor.enablturnLefte(self.timestep)
        self.rl_range_sensor.enable(self.timestep)
        self.rr_range_sensor.enable(self.timestep)
        """

    def frontDetection(self,valueFRleftS,valueFRright):
        no_action = (self.go_left == False and self.go_right == False)
        print(self.go_left, self.go_right)
        # Detection des bords pour contourner
        if(no_action and valueFRleftS < 1000 and valueFRleftS < valueFRright):
            self.go_left = True
            self.go_right = False
            print("hola")
        elif(no_action and valueFRright < 1000 and valueFRright < valueFRleftS):
            self.go_left = False
            self.go_right = True
            print("hola2")
        elif valueFRright >= 1000 and valueFRright >= 1000:
            self.go_left = False
            self.go_right = False

        if self.go_left:
            self.motors.turnLeft()
        elif self.go_right: 
            self.motors.turnRight()
        else:
            self.runStraight()
        

                                #X                #Y                 #Z
    # Point GPS à ralier [-45.003356381052015, 52.28051606042675, 0.1689372910961459]
    def runStraightToPoint(self,X,Y):
        
        pointToRallyX = -45.003356381052015
        pointToRallyY = 52.28051606042675
        if(Y <  pointToRallyY) :
            self.motors.runStraight()
        elif (X > pointToRallyX) :
            self.motors.turnLeft()
        elif(X == pointToRallyX and Y == pointToRallyY):
            self.motors.runStraight()    



# create the Robot instance.
robot = monRobot()

# get the time step of the current world.


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(robot.timestep) != -1:
    robot.motors.stop()
    #robot.motors.turnRight()
    robot.motors.runStraight()
    value1 = robot.capteur1.getValue()
    value2 = robot.capteur2.getValue()
    print("val 1 "+str(value1))
    print("val 2 "+str(value2))

    #robot.frontDetection(valueFRleftS,valueFRright)
    

    #pass

