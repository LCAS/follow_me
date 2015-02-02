#!/usr/bin/env python

import time
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult

i = 0#global var, holds the state of the robot
def callback(data):
        global i
        if data.status.status >2: #if the robot failed/succeded in reaching goal store the result
            i = data.status.status
            
def coords(x,y,w):#function to publish the movement coords to robot
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)#where it is publishing to
    message = PoseStamped()
    message.header.frame_id = '/map'
    message.pose.position.x = x
    message.pose.position.y = y
    message.pose.position.z = 0.0
    message.pose.orientation.x = 0.0
    message.pose.orientation.y = 0.0
    message.pose.orientation.z = 0.0
    message.pose.orientation.w = w
    pub.publish(message)
      
       
def robot_base():
    end = False
    rospy.init_node('robot_base', anonymous=True)
    r = rospy.Rate(0.3) # .3hz
    global i
    k=0# The index for cords
       
# Below is the open and read file   "/home/sinj/robot/src/base_move/script/cords.txt"
    cord=[]
    for line in open('/home/sinj/robot/src/base_move/script/cords.txt'):
       cord.append(np.array([float(val) for val in line.rstrip('\n').split(' ') if val != '']))
       
   
    print'goal coords: X:{} Y:{} W:{} and there are {} waypoints'.format(cord[0],cord[1],cord[2], (len(cord)/3))
    
    while not rospy.is_shutdown():
       # x,y,w = cord[k],cord[k+1],cord[k+2] # not sure why but could not get the message.pose.position.y = cord[k+1] to work, so using this way
        s=5 # seconds, used for wait
        if end == False:        
            if i <3:
                coords(cord[k],cord[k+1],cord[k+2])
                rospy.Subscriber("/move_base/result", MoveBaseActionResult, callback)
                print("keep moving to goal")  
            if i == 3:
                print'goal reached, waiting for {} seconds. at waypoint {}/{}'.format(s,((k+3)/3),(len(cord)/3))
                i = 2
                time.sleep(s)# waits for 4 seconds
                if len(cord)-1 > k+3: 
                    k = k +3 #move index to next set of cords
                    print'next goal coords: X:{} Y:{} W:{}'.format(cord[k],cord[k+1],cord[k+2])
                    print'moving to waypoint {}/{}'.format(((k+3)/3),(len(cord)/3))
                else:
                    print'the guiding has finished'
                    end = True
                    #rospy.signal_shutdown("program end")
                    
        if i == 4:
            print'failed, retry- not implemented'
            
        if end:
            if i <3:
                coords(1.0,1.0,1.0)
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
 
