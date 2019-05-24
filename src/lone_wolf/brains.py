#!/usr/bin/env python
import rospy

#from random import random
from math import atan2, pi, cos, sin
from time import time, sleep

from random import random

from geometry_msgs.msg import Pose2D
from graphical_client.msg import Pose2D_Array

from std_msgs.msg import String

from serial import Serial


SPEED = 15
ROT = 0.05

on_track_angle = pi/3

half_arc = pi/8
radius = 200
crash = 100

has_seen_obs = False

sim = True
xbee = True
if xbee:
    sp = Serial('/dev/ttyUSB0', 9600)

obstacles = []
#obpub = []
#ball = Pose2D(random()*2000-1000, random()*2000-1000, 0)
ball = Pose2D()
#p = Pose2D(random()*2000-1000, random()*2000-1000, random()*2*pi)
p = Pose2D()

t0 = time()
gather = True

#rotGoal = 
#moveGoal = 0  # distance from ball

def xec(exp):
    global p, ball, player
    if exp == 'w':
        p.x += SPEED * cos(switchCoord(p.theta))
        p.y += SPEED * sin(switchCoord(p.theta))
        p.theta += cRand(0.1)
    elif exp == 'r':
        p.x -= SPEED * cos(switchCoord(p.theta))
        p.y -= SPEED * sin(switchCoord(p.theta))
        p.theta += cRand(0.1)
    elif exp == 's':
        p.theta += ROT+cRand(0.01)
    elif exp == 'a':
        p.theta -= ROT+cRand(0.01)
    p.theta %= 2*pi
    player.publish(p)
    showTraj()


####################MAIN##################

def GO():
    done = False
    print("Godspeed!")
    while(not done):
        print("Where's that ball?")
        rad = absoluteAngleTo(ball)
        #print(rad, p.theta)
        rotateTo(rad)
        print("There it is")
        done = advance(ball.x, ball.y)
    print("SUCCESS")

#######################CONTROL FUNCS#################

def advance(x, y, tol=100, noise=10, override=False):
    old = dist(x, y, p.x, p.y)
    curr = old
    while curr > tol:
        d, obs = closestObstacle()    
        if d < crash:
            print("Gonna crash!")
            avoid(obs)
            return False
        if xbee:
            sp.write('w')
        if sim:
            xec('w')
        sleep(0.05)
        curr = dist(x, y, p.x, p.y)
        if not override and abs(relativeAngleTo(ball)) > on_track_angle: 
            if xbee:
                sp.write(' ')
            return False
        if old < curr - noise:
            if xbee:
                sp.write(' ')
            return False
        old = curr
    if xbee:
        sp.write(' ')
    return True

def backwards(r, tol=30, noise=10):
    x = p.x-cos(switchCoord(p.theta))*r
    y = p.y-sin(switchCoord(p.theta))*r
    
    old = dist(x, y, p.x, p.y)
    curr = old
    while curr>tol:
        if xbee:
            sp.write('r')
        if sim:
            xec('r')
        sleep(0.05)
        curr = dist(x, y, p.x, p.y)
        if(old < curr-noise):
            sp.write(' ')
            return
        old = curr
    if xbee:
        sp.write(' ')

def rotateTo(rad, tol = 2*pi/360*10): #relative angle = angDist(r)*getDir(r)
    while(angDist(rad)>tol):
        if getDir(rad):
            if xbee:
                sp.write('s')
            if sim:
                xec('s')
        else:
            if xbee:
                sp.write('a')
            if sim:
                xec('a')
        sleep(0.05)

def closestObstacle(r=radius, ha=half_arc): # r aprox 300
    rt = rayTrace(3, radius, ha)
    minD = 100000
    obs = None
    for point in rt:
        for o in obstacles:
            di = dist(point[0], point[1], o.x, o.y)
            if(di<minD):
                minD=di
                obs = o
    return minD, obs 
    
def avoid(obs):
    global has_seen_obs
    if has_seen_obs:
        print("...and another!")
        backwards(200)
        has_seen_obs = False
        print('phew..')
    else:
        has_seen_obs = True
        rad = relativeAngleTo(obs)
        ba = relativeAngleTo(ball)
        if ba>0:
            rotateTo(p.theta+pi/4+rad)
        else:
            rotateTo(p.theta-pi/4+rad)
        hyp = pow(pow(radius + crash, 2)*2, 0.5)
        advance(p.x+cos(switchCoord(p.theta))*hyp, p.y+sin(switchCoord(p.theta))*hyp, tol=70, override=True)
    

########################HELPER FUNCS####################            

def absoluteAngleTo(obj): ## how much turn to face obj. ans is between -pi and pi
    dx = obj.x - p.x
    dy = obj.y - p.y
    rad = atan2(dy, dx) # radians in normal coords
    return switchCoord(rad)   # transform to weird coords

def relativeAngleTo(obj):
    rad = angDist(absoluteAngleTo(obj))
    return rad*(1 if getDir(rad) else -1)    

def rayTrace(k, r, ha):
    rt = []
    for i in range(k):
        x = p.x+cos(switchCoord(p.theta))*r*(i+1)/k
        y = p.y+sin(switchCoord(p.theta))*r*(i+1)/k
        rt.append((x, y))
        x = p.x+cos(switchCoord(p.theta)+ha)*r*(i+1)/k
        y = p.y+sin(switchCoord(p.theta)+ha)*r*(i+1)/k
        rt.append((x, y))
        x = p.x+cos(switchCoord(p.theta)-ha)*r*(i+1)/k
        y = p.y+sin(switchCoord(p.theta)-ha)*r*(i+1)/k
        rt.append((x, y))
    return rt
        

def dist(x1, y1, x2, y2):
    return pow(pow(x1-x2, 2) + pow(y1-y2, 2), 0.5)

def angDist(rad):
    return min(abs(p.theta-rad), abs(p.theta+2*pi-rad), abs(rad+2*pi-p.theta))

def getDir(rad):
    u = rad - p.theta # between -3/2*pi and pi/2
    if u < -pi:
        u +=2*pi
    return u>0

def switchCoord(t):
    return pi/2-t

#########################UPDATE FUNCS###############
def updateBall(data):
    global ball
    if abs(ball.x - data.x) + abs(ball.y - data.y) > 5:
        ball = data
        print('ball', ball.x, ball.y)
        GO()
        #prT()


def updateObstacle(data):
    global gather
    if gather:
        x = data.x
        y = data.y
        for o in obstacles:
            if abs(o.x - x) + abs(o.y - y) < 10:
                break
        else:
            pose = Pose2D(x, y, 0)
            obstacles.append(pose)
            print(len(obstacles))

def updatePlayer(data):
    global p

    if abs(p.x - data.x) + abs(p.y - data.y) > 1 or abs(p.theta-data.theta) > 5*2*pi/360:
        # print('wttf')
        p = data
        print('player', p.x, p.y, p.theta)
        sleep(0.1)
        showTraj()
        # prT()
#######################SETUP###################

def listener():
    global traj, player, baller, gather, p, ball
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('middleman', anonymous=True)
    for i in range(11):
        rospy.Subscriber("b_r{}".format(i), Pose2D, updateObstacle)
    rospy.Subscriber("y_r0", Pose2D, updatePlayer)
    rospy.Subscriber("ball", Pose2D, updateBall)
    #rospy.Subscriber("traj", Pose2D_Array, updateTraj)
    traj = rospy.Publisher('/trajectory', Pose2D_Array, queue_size=10)
    player = rospy.Publisher('/y_r0', Pose2D, queue_size=10)
    baller = rospy.Publisher('/ball', Pose2D, queue_size=10)
    if sim:
        ball = Pose2D(cRand(2000), cRand(2000), 0)
        p = Pose2D(cRand(2000), cRand(2000), random()*2*pi)
        
    sleep(1)
    player.publish(p)
    baller.publish(ball)
    sleep(1)
    showTraj()
    print("Gathering...")
    # spin() simply keeps python from exiting until this node is stopped
    #t0 = time()
    sleep(7)
    gather = False
    print("Done!")
    print("obstacles: ")
    for i, o in enumerate(obstacles):
        print(o.x, o.y)
        #obpub.append(rospy.Publisher('b_r{}'.format(i), Pose2D, updateObstacle))
    GO()
    rospy.spin()

############################DEBUG####################
def cRand(r):
    return random()*2*r-r

def showTraj():
    arr = Pose2D_Array()
    arr.poses.append(p)
    t = Pose2D()
    t.x = p.x+cos(switchCoord(p.theta))*radius  # float(input("tr x: "))
    t.y = p.y+sin(switchCoord(p.theta))*radius  # float(input("tr y: "))
    t.theta = p.theta
    arr.poses.append(t)
    arr.poses.append(p)
    t = Pose2D()
    t.x = p.x+cos(switchCoord(p.theta)+half_arc)*radius  # float(input("tr x: "))
    t.y = p.y+sin(switchCoord(p.theta)+half_arc)*radius  # float(input("tr y: "))
    t.theta = p.theta
    arr.poses.append(t)
    arr.poses.append(p)
    t = Pose2D()
    t.x = p.x+cos(switchCoord(p.theta)-half_arc)*radius  # float(input("tr x: "))
    t.y = p.y+sin(switchCoord(p.theta)-half_arc)*radius  # float(input("tr y: "))
    t.theta = p.theta
    arr.poses.append(t)
    traj.publish(arr)


def prT():
    print('player', p.x, p.y, p.theta)

    dx = ball.x - p.x
    dy = ball.y - p.y
    rad = atan2(dy, dx)
    rad -= pi/2
    rad *= -1
    rad %= 2*pi
    deg = rad/2/pi*360
    update = rad - p.theta
    update %= 2*pi
    if update > pi:
        update = pi - update

    print('\nslope deg: ', deg)
    print('slope rad: ', deg*2*pi/360)
    print('must rot: ', deg - p.theta/2/pi*360)  # amount to rotate
    print('must rot: ', update, ' radians')

    showTraj()

    sleep(1)
    p.theta += update
    p.theta %= 2*pi
    showTraj()

    print('player', p.x, p.y, p.theta)


    for o in obstacles:
        o.x = round(o.x)
        o.y = round(o.y)
        o.theta = 0
        print('x: ', o.x, ', y: ', o.y)
    print(len(obstacles))
    # prT()



if __name__ == '__main__':
    listener()
