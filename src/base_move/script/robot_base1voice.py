#!/usr/bin/env python
import os
import time
import rospy
import random
import actionlib
import numpy as np
import mary_tts.msg
import move_base_msgs.msg
    
def makegoal(x,y,z,w): #makes move goal & return it 
    target = move_base_msgs.msg.MoveBaseGoal()
    target.target_pose.header.frame_id = "/map"
    target.target_pose.pose.position.x = x
    target.target_pose.pose.position.y = y
    target.target_pose.pose.orientation.z = z
    target.target_pose.pose.orientation.w = w 
    return target        
       
def robot_base():
    end = False
    rospy.init_node('robot_base', anonymous=True)
    r = rospy.Rate(0.3) # .3hz
    countstuck = 0  
    k=0# The index for cords
    speakfile = '/tospeak.txt'
    cordfile = '/cords6.txt'
    robotstate = 0
    s=5 # seconds, used for wait 
    
    #~~~~~~~~~~~~~~~
    dlist = []
    dlist.append('========== New participant ===============')
    startimer = time.time()
    counttry = 1
    idint = random.randint(1,200)
    dlist.append('~~~ ID num = {} ~~~'.format(idint))
    #~~~~~~~~~~~~~~~
    
    maryclient = actionlib.SimpleActionClient('speak',mary_tts.msg.maryttsAction)    
    maryclient.wait_for_server()
    baseClient = actionlib.SimpleActionClient('move_base',move_base_msgs.msg.MoveBaseAction)
    baseClient.wait_for_server()#
     #speaking goal    
    speak = mary_tts.msg.maryttsGoal()      
      
# Below is the open and read file   "/home/sinj/robot/src/base_move/script/cords.txt"
    cord=[]
    for line in open(os.path.dirname(os.path.realpath(__file__)) + cordfile): # takes the cord.txt from same directory
       cord.append(np.array([float(val) for val in line.rstrip('\n').split(' ') if val != '']))
    print "reading cords from {}".format(cordfile)
    print "reading speaking words from {}".format(speakfile)           
    with open(os.path.dirname(os.path.realpath(__file__)) + speakfile, "r") as f:
        speakarray = map(str.rstrip, f)   
   
    speak.text = speakarray[0]  # greet people              
    maryclient.send_goal_and_wait(speak)
    print'Going to start at coords: X:{} Y:{} Z:{} W:{} and there are {} waypoints'.format(cord[0],cord[1],cord[2],cord[3], (len(cord)/4)-1)
    speak.text = speakarray[1]  #inform them of action              
    maryclient.send_goal_and_wait(speak)
    #coords(cord[k],cord[k+1],cord[k+2],cord[k+3])
    baseClient.send_goal_and_wait(makegoal(cord[k],cord[k+1],
                                                       cord[k+2],cord[k+3]))
    speak.text = speakarray[2]   #ask them to move behind robot             
    maryclient.send_goal_and_wait(speak)                                                                                                                
    k = k +4 #move index to next set of cords
    time.sleep(s)# waits for 4 seconds
    
    while not rospy.is_shutdown():
        if end == False:        
            if robotstate <3:
                print("keep moving to goal")
                speak.text = speakarray[random.randint(3,7)]                
                maryclient.send_goal_and_wait(speak) 
                baseClient.send_goal_and_wait(makegoal(cord[k],cord[k+1],
                                                       cord[k+2],cord[k+3]))    #send goal cords
                robotstate = baseClient.get_state()#get the current state of robot
            if robotstate == 3:
                print'goal reached, waiting for {} seconds. at waypoint {}/{}'.format(s,((k+4)/4),(len(cord)/4))
                robotstate = 2
                speak.text = speakarray[random.randint(8,10)]                
                maryclient.send_goal_and_wait(speak)                
                time.sleep(s)# waits for X seconds                 
                if len(cord)-1 > k+4: 
                    k = k +4 #move index to next set of cords
                    print'next goal coords: X:{} Y:{} Z{}: W:{}'.format(cord[k],cord[k+1],cord[k+2],cord[k+3])
                    print'moving to waypoint {}/{}'.format(((k+4)/4),(len(cord)/4))
                else:
                    print'the guiding has finished'
                    end = True
                    
                    
        if robotstate == 4:
            print'failed, retry'
            if countstuck < 5:
                countstuck = countstuck + 1
                baseClient.send_goal_and_wait(makegoal(cord[k],cord[k+1],
                                                       cord[k+2],cord[k+3]))
                robotstate = baseClient.get_state()
                #~~~~~
                dlist.append("robot lost path {} times".format(counttry))
                counttry = counttry +1
                #~~~~~ 
            else:
                speak.text = "i am stuck, need help"                
                maryclient.send_goal_and_wait(speak)
                #~~~~~
                dlist.append("==========robot Stuck, need assestiance===========")
                dlist.append("exporting file in of reset\n ")
                with open(os.path.dirname(os.path.realpath(__file__)) + '/Method1_stats_stuck.txt','a') as File:
                    for item in dlist:
                        print>>File, item                 
                #~~~~~

        if end:
            if robotstate <3:
                speak.text = speakarray[random.randint(14,15)]                
                maryclient.send_goal_and_wait(speak)
                speak.text = speakarray[16]                
                maryclient.send_goal_and_wait(speak)
                print'moving robot back to start'
                baseClient.send_goal_and_wait(makegoal(-2.0,0.0,1.0,0.1))
                robotstate = baseClient.get_state() #get the current state of robot
            if robotstate == 3:
                 speak.text = speakarray[17]
                 maryclient.send_goal_and_wait(speak)
                 #~~~~~
                 dlist.append('guide end, took {:.2f} seconds'.format(startimer - time.time()))
                 dlist.append('guide end, took {:.2f} seconds'.format((startimer - time.time())/60))                 
                 dlist.append('=========== end ======== \n')
                 with open(os.path.dirname(os.path.realpath(__file__)) + '/Method1_stats.txt','a') as File:
                     for item in dlist:
                         print>>File, item
                 #~~~~~                            
                 print'program end'
                 rospy.signal_shutdown("program end")
                 
        r.sleep()
    
if __name__ == '__main__':
    try:
        robot_base()
    except rospy.ROSInterruptException: pass
 
