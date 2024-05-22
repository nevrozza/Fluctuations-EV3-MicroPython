#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# Инициализация
ev3 = EV3Brick()
topMotor = Motor(Port.A)
pushMotor = Motor(Port.B)
touchSensor = TouchSensor(Port.S1)
sonicSensor = UltrasonicSensor(Port.S3)
generalTimer = StopWatch()
fluctuaitonsTimer = StopWatch()

def printStartText():
    ev3.screen.clear()
    yOffset = ev3.screen.height/2 - 20
    xOffset = ev3.screen.width/2
    ev3.screen.draw_text(x = xOffset-6*6, y = yOffset-3, text = "Ready?")
    ev3.screen.draw_text(x = xOffset-6*5, y = yOffset+26, text = "Press", background_color=Color.BLACK, text_color=Color.WHITE)
    ev3.screen.draw_text(x = xOffset-6*12, y = yOffset+44, text = "central button", background_color=Color.BLACK, text_color=Color.WHITE)

def printProcessText():
    ev3.screen.clear()
    ev3.screen.draw_text(x = ev3.screen.width/2 - 6 * 10, y = ev3.screen.height/2 - 6, text = "in process...")


class Robot:
    def __init__(self):
        self.isRolling = False
        self.isStarted = False
        self.isResult = False
        self.isPaused = True
        self.isClawsInProcess = False
        self.nullDistance = 0
        self.prevDistance = 0
        self.sweepsCount = 0
        self.isInited = False
        # Фикс при зажатии кнопки
        self.centralButtonClickedTime = 0

def returnClawsToNull(robot):
    wait(1000)
    pushMotor.run_angle(1000, -pushMotor.angle())
    robot.isClawsInProcess = True

def startAnalysis(robot):
    robot.isStarted = True
    robot.isClawsInProcess = False
    robot.isInited = False
    # robot.nullDistance = int(sonicSensor.distance())

def resetRobotStates(robot):
    robot.isResult = False
    robot.isRolling = False
    robot.isStarted = False
    robot.isClawsInProcess = False
    robot.nullDistance = 0
    robot.prevDistance = 0
    robot.sweepsCount = 0

def updateCount(robot):
    robot.sweepsCount+=1
    analysisText = "analysis" + "."*(robot.sweepsCount%3+1)
    ev3.screen.clear()
    ev3.screen.draw_text(x = ev3.screen.width/2-6*len("analysis"), y = ev3.screen.height/2-6, text = analysisText)

def detectSweep(robot):
    if((robot.prevDistance >= robot.nullDistance and sonicSensor.distance() < robot.nullDistance)):
        if(not robot.isInited):
            robot.isInited = True
            fluctuaitonsTimer.reset()
        else:
            updateCount(robot)

def getT(robot):
    return 2*(22/7) * ((((robot.nullDistance-160)/1000)/9.81) ** (.5))
    # t = fluctuaitonsTimer.time()/1000
    # N = robot.sweepsCount/2
    # return t/N

def printResult(robot):
    T = getT(robot)
    robot.isRolling = False
    robot.isStarted = False
    robot.isClawsInProcess = False
    if(not robot.isResult):
        robot.isResult = True
        # L = (9.81 * T**2) / (4 * (22/7)**2)
        ev3.screen.clear()
        # ev3.screen.draw_text(x = ev3.screen.width/2-6*12, y = ev3.screen.height/2-6, text = "T: "+str(T)[1:5]+" ±0.05")
        ev3.screen.draw_text(x = ev3.screen.width/2-6*12, y = ev3.screen.height/2-6+14, text = "T: "+str(T)[0:5]+" ±0.05")
        ev3.screen.draw_text(x = ev3.screen.width/2-6*12, y = ev3.screen.height/2-6-3, text = "L: "+str((robot.nullDistance-160)/1000)[0:5]+" ±0.05")

def unpausedIdle(robot):
    robot.centralButtonClickedTime = generalTimer.time()
    robot.isPaused = False
    robot.isRolling = False
    topMotor.stop()
    
    ev3.screen.clear()
    ev3.screen.draw_text(x = ev3.screen.width/2-6*len("Press"), y = ev3.screen.height/2-6, text = "Press")
    ev3.screen.draw_text(x = ev3.screen.width/2-6*len("Red button!!"), y = ev3.screen.height/2-6+20, text = "Red button!!")

def rollUp(robot):
    topMotor.run(300)
    robot.isRolling = True

def rollDown(robot):
    topMotor.run(-300)
    robot.isRolling = True

def stopRolling(robot):
    topMotor.stop()
    robot.isRolling = False

printStartText()

robot = Robot()

while True:
    # ev3.screen.print(robot.nullDistance)
    # ev3.screen.print(robot.nullDistance)
    # Collection нажатых кнопок
    pB = ev3.buttons.pressed()


    # Возвращение "клешней" в изначальную позицию
    if(pushMotor.angle() <= -120):
        robot.nullDistance = sonicSensor.distance()
        returnClawsToNull(robot)

    # Начало подсчёта
    elif(pushMotor.angle() >= -80 and robot.isClawsInProcess):
        startAnalysis(robot)

    # Обработка нажатия на TouchSensor
    if(not robot.isStarted):
        if(touchSensor.pressed()):
            resetRobotStates(robot)
            robot.isPaused = False
            
            printProcessText()
            # Захват груза для запуска
            pushMotor.run_angle(50, -120)
            

    if(robot.isPaused):
        # Остановка прокрутки, если Up/Down кнопки не нажаты
        if(robot.isRolling and (Button.UP not in pB and Button.DOWN not in pB)):
            stopRolling(robot)

        # Обработка нажатых кнопок
        # Фикс проблемы с лишним нажатием, когда Central кнопка зажата (разница в нажатиях должна быть больше 1 сек)
        if(Button.CENTER in pB and generalTimer.time() - robot.centralButtonClickedTime > 1000):
            unpausedIdle(robot)

        elif(Button.UP in pB):
            rollUp(robot)

        elif(Button.DOWN in pB):
            rollDown(robot)
    else :
        if(Button.CENTER in pB and generalTimer.time() - robot.centralButtonClickedTime > 1000):
            robot.centralButtonClickedTime = generalTimer.time()
            resetRobotStates(robot)
            robot.isPaused = True
            printStartText()
        
        else:
            if(robot.sweepsCount == 5):
                printResult(robot)
            else: detectSweep(robot)            
                    
            robot.prevDistance = sonicSensor.distance()
            
        
        

    
