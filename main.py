#!/usr/bin/env pybricks-micropython
import sys

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

ev3 = EV3Brick()

motor_right = Motor(Port.D)
motor_left = Motor(Port.A)

ev3.speaker.beep()




obstacle = [[0.915, 0.305], [0.915, 0.61], [0.915, 0.915], [0.915, 1.22], [0.915, 1.525], [1.829, 1.22], [1.829, 1.525], [1.829, 1.829], [1.829, 2.134], [1.829, 2.439], [1.829, 2.743], [3.048, 1.22], [3.048, 1.525], [3.048, 1.829], [3.353, 0.915], [3.353, 1.22], [3.658, 0.915], [3.658, 1.22]]

start = [0.305, 0.61]
goal = [3.658, 1.829]
tile = 0.305
startScaled = [round(start[0] / tile), round(start[1] / tile)]

grid = [[None]*16 for i in range(10)]

goalScaled = [round(goal[0] / tile), round(goal[1] / tile)]

grid[goalScaled[1]-1][goalScaled[0]-1] = 0
grid[goalScaled[1]-1][goalScaled[0]] = 0
grid[goalScaled[1]][goalScaled[0]-1] = 0
grid[goalScaled[1]][goalScaled[0]] = 0

obstacleScaled = []

for idx, obs in enumerate(obstacle):
    if(obs[0] >= 0 and obs[1] >= 0):
        obstacleCenterX = round(obs[0] / tile)
        obstacleCenterY = round(obs[1] / tile)
        obstacleScaled.insert(idx, [obstacleCenterX, obstacleCenterY])

for idx, obs in enumerate(obstacleScaled):
    grid[obs[1]-1][obs[0]-1] = sys.maxsize
    grid[obs[1]-1][obs[0]] = sys.maxsize
    grid[obs[1]][obs[0]-1] = sys.maxsize
    grid[obs[1]][obs[0]] = sys.maxsize

queue = []

def Manhattan(i, j): 
    min = sys.maxsize
    if i < 9:
        if grid[i+1][j] != None:
            if (grid[i+1][j] + 1) < min:
                min = grid[i+1][j] + 1
        else:
            queue.append((i+1, j))
    if i > 0:
        if grid[i-1][j] != None:
            if (grid[i-1][j] + 1) < min:
                min = grid[i-1][j] + 1
        else:
            queue.append((i-1, j))
    if j > 0:
        if grid[i][j-1] != None:
            if (grid[i][j-1] + 1) < min:
                min = grid[i][j-1] + 1
        else:
            queue.append((i, j-1))
    if j < 15:
        if grid[i][j+1] != None:
            if (grid[i][j+1] + 1) < min:
                min = grid[i][j+1] + 1
        else:
            queue.append((i, j+1))
    grid[i][j] = min
    
def StartManhattan(goalGridIndices):
    start = None
    if goalGridIndices[1] < 15:
        if grid[goalGridIndices[0]][goalGridIndices[1] + 1] == None:
            start = (goalGridIndices[0], goalGridIndices[1] + 1)
    if goalGridIndices[1] > 0:
        if grid[goalGridIndices[0]][goalGridIndices[1] - 1] == None:
            start = (goalGridIndices[0], goalGridIndices[1] - 1)
    if goalGridIndices[0] < 9:
        if grid[goalGridIndices[0]+1][goalGridIndices[1]] == None:
            start = (goalGridIndices[0] + 1, goalGridIndices[1])
    if goalGridIndices[1] > 0:
        if grid[goalGridIndices[0]-1][goalGridIndices[1]] == None:
            start = (goalGridIndices[0] - 1, goalGridIndices[1])
    if start == None:
        return False
    else:
        queue.append(start)
        while len(queue) != 0:
            cell = queue.pop(0)
            Manhattan(cell[0], cell[1])
        return True
        
def StartManhattanHelper(goalCoordinates):
    goalGridIndices = [(goalCoordinates[1], goalCoordinates[0]), (goalCoordinates[1], goalCoordinates[0]-1), (goalCoordinates[1]-1, goalCoordinates[0]), (goalCoordinates[1]-1, goalCoordinates[0]-1)]
    for i in range(4):
        if StartManhattan(goalGridIndices[i]) == True:
            break
        

StartManhattanHelper(goalScaled)

for y in range(9, -1, -1):
    for x in range(16):
        if grid[y][x] == sys.maxsize:
            print("obs", end = "\t")
        else:
            print(grid[y][x], end = "\t")
    print()

def FindStartCell(startCoordinates):
    startGridIndices = [(startCoordinates[1], startCoordinates[0]), (startCoordinates[1], startCoordinates[0]-1), (startCoordinates[1]-1, startCoordinates[0]), (startCoordinates[1]-1, startCoordinates[0]-1)]
    minValue = sys.maxsize
    minIndices = None
    minQuadrant = 0
    iteration = 1
    for indices in startGridIndices:
        if grid[indices[0]][indices[1]] < minValue:
            minValue = grid[indices[0]][indices[1]]
            minIndices = indices
            minQuadrant = iteration
        iteration += 1
    return(minIndices, minQuadrant)

def GoToStartCell(quadrant):
    # robot placed facing right
    if quadrant == 3:
        print("quadrant 3")
        motor_left.run_angle(200, 344, wait = False) # go forward
        motor_right.run_angle(200, 344) # go forward
        motor_left.run_angle(200, 180, wait = False) # turn right
        motor_right.run_angle(200, -180) # turn right
        motor_left.run_angle(200, 344, wait = False) # go forward
        motor_right.run_angle(200, 344) # go forward
        motor_right.run_angle(200, 180, wait = False) # turn left
        motor_left.run_angle(200, -180) # turn left
    elif quadrant == 1:
        motor_left.run_angle(200, 344, wait = False) # go forward
        motor_right.run_angle(200, 344) # go forward
        motor_right.run_angle(200, 180 - 2, wait = False) # turn left
        motor_left.run_angle(200, -180 + 2) # turn left
        motor_left.run_angle(200, 344, wait = False) # go forward
        motor_right.run_angle(200, 344) # go forward
        motor_left.run_angle(200, 180, wait = False) # turn right
        motor_right.run_angle(200, -180) # turn right
    elif quadrant == 4:
        motor_left.run_angle(200, -344, wait = False) # go backwards
        motor_right.run_angle(200, -344) # go backwards
        motor_left.run_angle(200, 180, wait = False) # turn right
        motor_right.run_angle(200, -180) # turn right
        motor_left.run_angle(200, 344, wait = False) # go forward
        motor_right.run_angle(200, 344) # go forward
        motor_right.run_angle(200, 180, wait = False) # turn left
        motor_left.run_angle(200, -180) # turn left
    elif quadrant == 2:
        motor_left.run_angle(200, -344, wait = False) # go backwards
        motor_right.run_angle(200, -344) # go backwards
        motor_right.run_angle(200, 180, wait = False) # turn left
        motor_left.run_angle(200, -180)
        motor_left.run_angle(200, 344, wait = False) # go forward
        motor_right.run_angle(200, 344) # go forward
        motor_left.run_angle(200, 180, wait = False) # turn right
        motor_right.run_angle(200, -180)

def Navigate(start, orientation):
    NavigateFrom(start[0], start[1], orientation)

def NavigateFrom(i, j, orientation):
    if(grid[i][j] == 0):
        print("goal reached!")
        return
    minValue = sys.maxsize
    minIndices = None
    minOrientation = None
    if i < 9:
        if grid[i+1][j] != None:
            if (grid[i+1][j]) < minValue:
                minValue = grid[i+1][j]
                minIndices = (i+1, j)
                minOrientation = 90 # up
    if i > 0:
        if grid[i-1][j] != None:
            if (grid[i-1][j]) < minValue:
                minValue = grid[i-1][j] 
                minIndices = (i-1, j)
                minOrientation = 270 # down
    if j > 0:
        if grid[i][j-1] != None:
            if (grid[i][j-1]) < minValue:
                minValue = grid[i][j-1]
                minIndices = (i, j-1)
                minOrientation = 180 # left
    if j < 15:
        if grid[i][j+1] != None:
            if (grid[i][j+1]) < minValue:
                minValue = grid[i][j+1]
                minIndices = (i, j+1)
                minOrientation = 0 # right
    if minIndices == None:
        print("no path exists")
    else:
        print(minValue, minOrientation)
        Turn(orientation, minOrientation)
        GoForward()
        NavigateFrom(minIndices[0], minIndices[1], minOrientation)
    
# Assume start orientation is 0 degrees aka right 

def Turn(prevOrientation, newOrientation):
    difference = newOrientation - prevOrientation
    if difference == 270:
        difference = -90
    elif difference == -270:
        difference = 90
    if difference > 0:
        motor_right.run_angle(100, (difference*4 - 4)/2, wait = False) # turn left
        motor_left.run_angle(100, -(difference*4 - 4)/2)
    if difference < 0:
        motor_left.run_angle(100, -(difference*4 + 2)/2, wait = False) # turn right
        motor_right.run_angle(100, (difference*4 + 2)/2)

def GoForward():
    motor_left.run_angle(200, 650, wait = False) 
    motor_right.run_angle(200, 650)

startCell = FindStartCell(startScaled)
print(startCell[0], startCell[1])
GoToStartCell(startCell[1])
ev3.speaker.beep(frequency = 1000, duration = 500)
Navigate(startCell[0], 0)
ev3.speaker.beep(frequency = 1000, duration = 500)

