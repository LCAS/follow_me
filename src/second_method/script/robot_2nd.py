#!/usr/bin/env python

import time
import math
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped 
from move_base_msgs.msg import MoveBaseActionResult
sqrt = 0.0
i = 0#global var, holds the state of the robot
def callback(data):
        global i
        if data.status.status >2: #if the robot failed/succeded in reaching goal store the result
            i = data.status.status
            
def coords(x,y,z,w):
    #function to publish the movement coords to robot
    
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)#where it is publishing to
    message = PoseStamped()
    message.header.frame_id = '/map'
    message.pose.position.x = x
    message.pose.position.y = y
    message.pose.orientation.z = z
    message.pose.orientation.w = w
    pub.publish(message)
    
    
def wherehuman(data):
 #get the distance the humans is away from  the robot and store it  
    global sqrt
    toBeSqrt = (data.pose.position.x * data.pose.position.x) + (data.pose.position.y * data.pose.position.y)   
    sqrt = math.sqrt(toBeSqrt) 
  
def robot_base():
   
    rospy.init_node('robot_base', anonymous=True)
    r = rospy.Rate(0.3) # .3hz
    global i
    global sqrt
    k=0# The index for cords
    end = False   #signal the end for the program
    humanDistance = 4  # stores max distance the human can be from the robot
    waitTime = 20.0    #in seconds
  
# Below is the open and read file
    cord=[]
    for line in open('/home/sinj/robot/src/second_method/script/cords3.txt'):
       cord.append(np.array([float(val) for val in line.rstrip('\n').split(' ') if val != '']))
       
    
    print'goal coords: X:{} Y:{} Z:{} W:{} and there are {} waypoints'.format(cord[0],cord[1],cord[2],cord[3], (len(cord)/4))
    
    while not rospy.is_shutdown():       
                    
        if end == False:        
            if i <3:
                coords(cord[k],cord[k+1],cord[k+2], cord[k+3])
                rospy.Subscriber("/move_base/result", MoveBaseActionResult, callback)
                print("keep moving to goal")  
                
            if i == 3:
                print'goal reached, waiting for human. \nCurrently at waypoint {}/{}'.format(((k+4)/4),(len(cord)/4))
                sqrt = 0.0
                waitForHuman = True
                timer = time.time()
                i=2
                while waitForHuman:                   
                                       
                    print "waiting for {:.2f} seconds ".format(((timer+waitTime) - time.time()))
                    rospy.Subscriber("/human/transformed", PoseStamped, wherehuman)
                    r.sleep()
                                        
                    if sqrt >0.5 and sqrt <= humanDistance:
                        print 'human spotted at {}'.format(sqrt)
                        if len(cord)-1 > k+4: 
                           sqrt = 0.0
                           k = k +4 #move index to next set of cords
                           print'next goal coords: X:{} Y:{} Z:{} W:{}'.format(cord[k],cord[k+1],cord[k+2],cord[k+3])
                           print'moving to waypoint {}/{}'.format(((k+4)/4),(len(cord)/4))                          
                        else:
                            print'the guiding has finished'
                            end = True                        
                        waitForHuman = False
                        
                    elif (timer+waitTime) < time.time():
                         print "human has not appered after {} seconds".format(waitTime)
                         r.sleep()
                         if  k-4 >= 0:
                             print"Robot will go back to prior waypoint and wait"
                             k = k -4 #move index to prior set of cords
                             print'next goal coords: X:{} Y:{} Z:{} W:{}'.format(cord[k],cord[k+1],cord[k+2],cord[k+3])
                             print'moving back to prior waypoint {}/{}'.format(((k+4)/4),(len(cord)/4))
                         else:
                             print 'no human is following robot, will go back to start' 
                             end = True
                         waitForHuman = False
                    else: 
                        print 'waiting for human'               
                                      
        if i == 4:
            print'failed, retry- not implemented'
            
        if end:
            if i <3:
                coords(1.0,1.0,1.0,0.1)
                rospy.Subscriber("/move_base/result", MoveBaseActionResult, callback)
                print'moving robot back to start'
            if i == 3:
                 print'program end'
                 rospy.signal_shutdown("program end")                 
        r.sleep()
    
if __name__ == '__main__':
    try:
        robot_base()
    except rospy.ROSInterruptException: pass
