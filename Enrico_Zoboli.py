#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ENPM 661 - Dr. Reza Monfaredi
Project 3 Part 1
Due date: March 21, 11:59 p.m.

@author: jcheng and ezoboli
"""
import numpy as np
import math
import sys
from collision import Vector, Poly, Concave_Poly, Circle, collide
import pygame as pg
import time

# global user inputs
start = [20,20,0]
goal = [150,150,0]
initial_distance = math.sqrt((goal[0]-start[0])**2 + (goal[1]-start[1])**2)
clearance = 5
robot_radius = 10
clear_Radius = clearance + robot_radius
step = 10
dTheta = 30

#vector declaration for collision
v = Vector

hexagon = Poly(v(0,0), [v(160,127), v(200,103), v(240,127), v(240,173), v(200,196),  v(160,173)])
poly = Concave_Poly(v(0,0), [v(36,65), v(144, 26), v(86, 67), v(117, 168)])
circle = Circle(v(300,65), 40)

#create list of all nodes (Node Structure: ownID, parentID, position, angle, cost)
nodes = []
openNodes = []
closedNodes = []
solution = []
tempSol = []
final_Node = {}

#create 3D array for all visited notes
visitedNodes = np.zeros((801, 501, 13))

#flag for complete search
completed = False

#=============================================================== FUNCTIONS DEFINITIONS ==========================================

# Check for obstacle collision
def checkDistance(x, y):
    robot = Circle(v(x,y), clear_Radius)
    if collide(robot, hexagon) or collide(robot, poly) or collide(robot, circle):
        return True    
    elif x <= clear_Radius or x>= 400-clear_Radius or y <= clear_Radius or y >= 250-clear_Radius:
        return True
    else:
        return False

# Check if goal is reached
def checkGoal(distance, angle):
    global goal, clear_Radius
    
    if distance < 1.5*clear_Radius and angle == goal[2]:
        print("Reached Target...")
        return True
    else:
        return False
        
# Intake user input
def getCoordinate(flag):
    global start, clearance, robot_radius, initial_distance, step, dTheta
    # Print out start or goal commands
    if flag:
        string = "starting"
    else:
        string = "goal"
    
    while True:
              
        # Check if within width bounds
        xBound = range(clear_Radius, 400-clear_Radius)
        upperX = str(400-clear_Radius)
        x = int(input("Enter "+string+" X coordinate: \n(Must be between "+str(clear_Radius)+" and "+upperX+"): \n"))
        while True:
            if x in xBound:
                break
            else:
                x = int(input("Error. Please enter an X between "+str(clear_Radius)+" and "+upperX+".\nEnter "+string+" X coordinate:"))
        
        # Check if within height bounds
        yBound = range(clear_Radius, 250-clear_Radius)
        upperY = str(250-clear_Radius)
        y = int(input("Enter "+string+" Y coordinate: \n(Must be between "+str(clear_Radius)+" and "+upperY+"): \n"))
        while True:
            if y in yBound:
                break
            else:
                y = int(input("Error. Please choose a Y between "+str(clear_Radius)+" and "+upperY+".\nEnter "+string+" Y coordinate:"))
        
        # Check if within height bounds
        angle = int(input("Enter "+string+" angle degree: \n"))
        while True:
            if angle % dTheta == 0.0:
                break
            else:
                angle = int(input("Error. Please choose angle that is divisible by "+str(dTheta)+".\nEnter "+string+" angle degree:"))
        if abs(angle % 360) > 180:
            if angle > 0:
                angle = (angle % 360) - 360
            else:
                angle = (angle % 360) + 360
        else:
            angle = angle % 360
        
        # Check if the point is over an object
        if not checkDistance(x,y):
            break
        else:
            print("Point is on an object. Please enter corrdinates again.\n\n")
    return[x, 250-y, angle]

# Check new node after movement
def action(node, deltaAngle):
    global completed, final_Node, goal
    ang = node['angle'] + deltaAngle
    
    #normalize angle
    if ang >= 360:
        ang = ang % 360
    if ang < 0:
        ang = 360 + ang
        
    x = node['pos'][0] + math.cos(math.radians(ang))*step
    y = node['pos'][1] - math.sin(math.radians(ang))*step
    parentID = node['ownID']
    cost = node['cost'] + 1
    round_x = round(x*2)
    round_y = round(y*2)
    goal_distance = math.sqrt((goal[0]-x)**2 + (goal[1]-y)**2)
    
    #check if we reached the goal
    completed = checkGoal(goal_distance, ang)
    if completed:
        final_Node = {'ownID': len(nodes),
                    'parentID': parentID,
                    'pos': [x,y],
                    'angle': ang,
                    'cost': cost,
                    'cost2': cost + goal_distance}
    
    #if we haven't visited this node yet, and we're clear, then create it and add it to the list
    if visitedNodes[round_x][round_y][int(ang/30)] == 0 and checkDistance(x,y) == False:
        new_Node = {'ownID': len(nodes),
                    'parentID': parentID,
                    'pos': [x,y],
                    'angle': ang,
                    'cost': cost,
                    'cost2': cost + goal_distance}
        
        nodes.append(new_Node)
        openNodes.append(new_Node)
        visitedNodes[round_x][round_y][int(ang/30)] = 1
        
        return new_Node
    
    # if none of the conditions are met, just return the node passed
    return node

## 5 Move Functions

def move0(node):
    
    return action(currentNode, 0)
    
def move30(node):
    
    return action(currentNode, dTheta)
    
def move60(node):
    
    return action(currentNode, dTheta*2)
    
def moveNeg30(node):
    
    return action(currentNode, -dTheta)
    
def moveNeg60(node):

    return action(currentNode, -dTheta*2)
    
## Backtrack Function

def backtrack(startNode):
    global nodes, openNodes, closedNodes, solution, tempSol, final_Node
    
    # Make sure final goal node is appended
    currentNode = final_Node

    while currentNode['parentID'] != 0:
        solution.append(currentNode)
        currentNode = nodes[currentNode['parentID']]
        
    # Make sure origin is appended    
    solution.append(startNode)
    
    # Reverse solution
    solution.reverse()
    
    # Initialize screen
    SCREENSIZE = (400,250)
    screen = pg.display.set_mode(SCREENSIZE, pg.DOUBLEBUF|pg.HWACCEL)
    clock = pg.time.Clock()
    prev_x, prev_y = startingNode['pos']
    prev_x1, prev_y1 = startingNode['pos']
    pg.draw.circle(screen, (0,0,255), (prev_x, prev_y), 5, 1)
    pg.draw.circle(screen, (255,0,0), (goal[0], goal[1]), 1.5*clear_Radius, 1)
    pg.draw.circle(screen, (255,0,0), (goal[0], goal[1]), 1, 1)
    pg.draw.polygon(screen, (255,255,255), hexagon.points, 1)
    pg.draw.polygon(screen, (255,255,255), poly.points, 1)
    pg.draw.circle(screen, (255,255,255), (300,65), 40, 1)
    
    # Draw all nodes
    for i in nodes:
        currentNode = i
        while currentNode['parentID'] != 0:
            tempSol.append(currentNode)
            currentNode = nodes[currentNode['parentID']]
        
        for i in tempSol:
            x, y = i['pos']
            prev_x, prev_y = nodes[i['parentID']]['pos']
            for event in pg.event.get():
                if event.type == pg.QUIT:
                    sys.exit() 
            
            pg.draw.line(screen, (0,255,255), (int(prev_x), int(prev_y)), (round(x), round(y)), 1)
            pg.display.flip()
            
            clock.tick(6000)   
            
        tempSol = []
        
    # Draw solution
    for i in solution:
        
        x, y = i['pos']
    
        for event in pg.event.get():
            if event.type == pg.QUIT:
                sys.exit() 
        
        pg.draw.line(screen, (0,0,255), (int(prev_x), int(prev_y)), (round(x), round(y)), 3)
        pg.draw.circle(screen, (0,0,255), (x,y), 4, 3)
        pg.display.flip()
        
        clock.tick(10)   
        prev_x, prev_y = i['pos']
    
    # Close Pygame
    time.sleep(10)
    pg.display.quit()
    
#==========================================================START OF CODE========================================================

#get all the user's input
clearance = int(input("Enter robot's clearance: \n"))
robot_radius = int(input("Enter robot's radius: \n"))
step = int(input("Enter step size: \n"))
#dTheta = int(input("Enter change in angle: \n"))
dTheta = 30
print("Angle units/steps will each be 30 degrees each.")
start = getCoordinate(True)
goal = getCoordinate(False)
initial_distance = math.sqrt((goal[0]-start[0])**2 + (goal[1]-start[1])**2)  

print("\nLoading...\n")

#create starting node
startingNode = {'ownID': 0,
                'parentID': 0,
                'pos': [start[0], start[1]],
                'angle': start[2],
                'cost': 0,
                'cost2': initial_distance}

nodes.append(startingNode)
openNodes.append(startingNode)
prevNode = startingNode

#find all nodes and reach target
while completed == False:
    
    newList = sorted(openNodes, key=lambda d:d['cost2'], reverse = False)
    currentNode = newList[0]
    openNodes.remove(currentNode)
    closedNodes.append(currentNode)
    
    moveNeg60(currentNode)
    if completed:
        break
    moveNeg30(currentNode)
    if completed:
        break
    move0(currentNode)
    if completed:
        break
    move30(currentNode)
    if completed:
        break
    move60(currentNode)
    if completed:
        break
    
    # if openNodes is empty at any point in this loop, it means that the starting node is
    # blocked off from the goal, most likely because it is forced to run into walls or obstacles
    if not openNodes:
        print("Cannot reach goal based on starting position/direction and robot dimensions/clearances.")
        print("Please provide new starting and goal nodes.")
        start = getCoordinate(True)
        goal = getCoordinate(False)
        initial_distance = math.sqrt((goal[0]-start[0])**2 + (goal[1]-start[1])**2)  
        
        #create starting node
        startingNode = {'ownID': 0,
                        'parentID': 0,
                        'pos': [start[0], start[1]],
                        'angle': start[2],
                        'cost': 0,
                        'cost2': initial_distance}
        
        nodes.append(startingNode)
        openNodes.append(startingNode)
        prevNode = startingNode
    
#back track
print("\nGenerating Backtrack Visualization...\n")
backtrack(startingNode)

print("\n-------\nDone!")