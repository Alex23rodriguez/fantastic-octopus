#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Pose2D
from graphical_client.msg import Pose2D_Array

from math import cos, sin, pi
from random import random
from time import sleep

obcount = 0

p = Pose2D()
b = Pose2D()


def updatePlayer(data):
    global p
    p = data


def updateBall(data):
    global b
    b = data


def init_pose():
    pose = Pose2D()
    pose.x = 0
    pose.y = 0
    pose.theta = 0
    return pose


def showT(t):
    return -(t-pi/2)


def talker():
    global obcount
    rospy.Subscriber("y_r0", Pose2D, updatePlayer)
    rospy.Subscriber("ball", Pose2D, updateBall)
    traj = rospy.Publisher('/trajectory', Pose2D_Array, queue_size=10)
    ball = rospy.Publisher('/ball', Pose2D, queue_size=10)
    pl = rospy.Publisher('/y_r0', Pose2D, queue_size=10)
    
    obs = []
    for i in range(11):
        obs.append(rospy.Publisher('/b_r{}'.format(i), Pose2D, queue_size=30))

    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1)  # 10hz
    aux = 1

    while not rospy.is_shutdown():
        i = raw_input("choose p or b or t or o: ")
        if i == "p":
            p.x = float(input("pl x: "))
            p.y = float(input("pl y: "))
            p.theta = float(input('pl t: '))

            pl.publish(p)
            arr = Pose2D_Array()
            arr.poses.append(p)
            t = Pose2D()
            t.x = p.x+cos(showT(p.theta))*250  # float(input("tr x: "))
            t.y = p.y+sin(showT(p.theta))*250  # float(input("tr y: "))
            t.theta = p.theta

            arr.poses.append(t)
            # print "The array is:", arr
            traj.publish(arr)
        elif i == 'b':
            b.x = float(input("ball x: "))
            b.y = float(input("ball y: "))
            ball.publish(b)
        elif i == 't':
            p.theta += float(input("dt: "))*2*pi/360  # deg to rad
            pl.publish(p)
        elif i == 'o':
            x = random()*2000-1000
            y = random()*2000-1000

            obs[obcount].publish(Pose2D(x, y, 0))
            obs[obcount].publish(Pose2D(x, y, 0))
            obs[obcount].publish(Pose2D(x, y, 0))
            obcount+=1
            obcount%=5
        elif i == 'br':
            b = Pose2D(random()*2000-1000, random()*2000-1000, 0)
            ball.publish(b)
        elif i == 'pr':
            p = Pose2D(random()*2000-1000, random()*2000-1000, random()*2*pi)
            pl.publish(p)
        elif i == 'c':
            for o in obs:
                o.publish(Pose2D(10000,10000,0))
                o.publish(Pose2D(10000,10000,0))
                o.publish(Pose2D(10000,10000,0))
            

            #arr = Pose2D_Array()
            # arr.poses.append(p)
            #t = Pose2D()
            # t.x = p.x+cos(showT(p.theta))*250  # float(input("tr x: "))
            # t.y = p.y+sin(showT(p.theta))*250  # float(input("tr y: "))
            #t.theta = p.theta
            # arr.poses.append(t)
            # traj.publish(arr)

    '''
    while not rospy.is_shutdown():
        arr = Pose2D_Array()
        for i in range(10):
            pose = init_pose()
            pose.x = 100 * ( i + 1 )
            pose.y = 150 * ( i + 1 ) * aux
            pose.theta +=0.7853 * i
            arr.poses.append(pose)
            aux *= -1
        print "The array is:", arr
        traj.publish(arr)
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        # traj.publish(hello_str)
        rate.sleep()
    '''


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
