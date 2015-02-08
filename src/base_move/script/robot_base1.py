#!/usr/bin/env python
import os
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
            
def coords(x,y,z,w):#function to publish the movement coords to robot
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)#where it is publishing to
    message = PoseStamped()
    message.header.frame_id = '/map'
    message.pose.position.x = x
    message.pose.position.y = y
    message.pose.orientation.z = z
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
    for line in open(os.path.dirname(os.path.realpath(__file__)) +'/cords.txt'): # takes the cord.txt from same directory
       cord.append(np.array([float(val) for val in line.rstrip('\n').split(' ') if val != '']))
       
   
    print'goal coords: X:{} Y:{} Z:{} W:{} and there are {} waypoints'.format(cord[0],cord[1],cord[2],cord[3], (len(cord)/4))
    
    while not rospy.is_shutdown():
        
        s=5 # seconds, used for wait
        if end == False:        
            if i <3:
                coords(cord[k],cord[k+1],cord[k+2],cord[k+3])
                rospy.Subscriber("/move_base/result", MoveBaseActionResult, callback)
                print("keep moving to goal")  
            if i == 3:
                print'goal reached, waiting for {} seconds. at waypoint {}/{}'.format(s,((k+4)/4),(len(cord)/4))
                i = 2
                time.sleep(s)# waits for 4 seconds
                print'k value before incroment {} and Lencord value {}'.format(k,len(cord))
                if len(cord)-1 > k+4: 
                    k = k +4 #move index to next set of cords
                    print'next goal coords: X:{} Y:{} Z{}: W:{}'.format(cord[k],cord[k+1],cord[k+2],cord[k+3])
                    print'moving to waypoint {}/{}'.format(((k+4)/4),(len(cord)/4))
                else:
                    print'the guiding has finished'
                    end = True
                    
                    
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
 
