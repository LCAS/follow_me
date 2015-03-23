#!/usr/bin/env python
import os
import flir_pantilt_d46.msg
import mary_tts.msg
import time
import math
import rospy
import random
import actionlib
import numpy as np
import move_base_msgs.msg 
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped 

sqrt = 0.0 # hold the sqrt distance of the human from the robot
    
def makegoal(x,y,z,w): #makes move goal & return it 
    target = move_base_msgs.msg.MoveBaseGoal()
    target.target_pose.header.frame_id = "/map"
    target.target_pose.pose.position.x = x
    target.target_pose.pose.position.y = y
    target.target_pose.pose.orientation.z = z
    target.target_pose.pose.orientation.w = w 
    return target  

def wherehuman(data):
    #get the euclidean distance the humans is away from  the robot and store it  
    global sqrt
    toBeSqrt = (data.pose.position.x * data.pose.position.x) + (data.pose.position.y * data.pose.position.y)   
    sqrt = math.sqrt(toBeSqrt)
          
def rotatehead(x):
     pub = rospy.Publisher('/head/commanded_state', JointState)       
     head_command = JointState() 
     head_command.name=["HeadPan"]
     if x > 0:
         head_command.position=[0.0] #forwards    
     else:
         head_command.position=[-180.0] #backwards
     pub.publish(head_command)
     
def robot_base():
    rospy.init_node('robot_base', anonymous=True)
    r = rospy.Rate(0.3) # .3hz
    cordsfile = '/cords6.txt'
    speakfile = '/tospeak.txt'
    global sqrt             # use the global var
    robotstate = 0
    countstuck = 0         # holds the state of the robot
    k=0                     # The index for cords
    end = False             # signal for the end for the program
    seekinghuman = True     # moving between waypoint should you looking for a humman
    out = 0                 # when to leave seeking token
    humanMaxDistance = 3.5  # stores max distance the human can be from the robot
    humanMinDistance = 0.5  # stores min distance the human can be from the robot
    robotWaitTime = 15.0    # in seconds
    robotquickWaitTime = 5  # in seconds
    
    #~~~~~~~~~~~~~~~
    dlist = []
    dlist.append('========== New participant ===============')
    startimer = time.time()
    count = 1
    countf = 1
    counttry = 1
    countseek =1
    idint = random.randint(1,200)
    dlist.append('~~~ ID num = {} ~~~'.format(idint))
    #~~~~~~~~~~~~~~~
    
        #make client
    print"making move clients"
    baseClient = actionlib.SimpleActionClient('move_base',move_base_msgs.msg.MoveBaseAction)
    baseClient.wait_for_server()#
    print"making ptu clients"        #make PTU client
    ptuclient = actionlib.SimpleActionClient('SetPTUState',flir_pantilt_d46.msg.PtuGotoAction)    
    ptuclient.wait_for_server()
    print"making speaking clients"       #Make speaking client
    maryclient = actionlib.SimpleActionClient('speak',mary_tts.msg.maryttsAction)    
    maryclient.wait_for_server()
        #face foward PTU goal 
    ptuforward = flir_pantilt_d46.msg.PtuGotoGoal()
    ptuforward.pan = 0
    ptuforward.pan_vel = 60
        #face backward PTU goal
    ptubwrd = flir_pantilt_d46.msg.PtuGotoGoal()
    ptubwrd.pan = -180
    ptubwrd.pan_vel = 60
        #speaking goal    
    speak = mary_tts.msg.maryttsGoal()    

        # Below is the open and read file- first as float, second as string
    print "reading cords from {}".format(cordsfile)
    cord=[]
    for line in open(os.path.dirname(os.path.realpath(__file__)) + cordsfile):
       cord.append(np.array([float(val) for val in line.rstrip('\n').split(' ') if val != '']))
       
    print "reading speaking text from {}".format(speakfile)           
    with open(os.path.dirname(os.path.realpath(__file__)) + speakfile, "r") as f:
        speakarray = map(str.rstrip, f)        
            
    speak.text = speakarray[0]  # great people              
    maryclient.send_goal_and_wait(speak)
    print'Going to start at coords: X:{} Y:{} Z:{} W:{} and there are {} waypoints'.format(cord[0],cord[1],cord[2],cord[3], (len(cord)/4))
    speak.text = speakarray[1]  #inform them of action              
    maryclient.send_goal_and_wait(speak) 
    baseClient.send_goal_and_wait(makegoal(cord[k],cord[k+1],
                                                       cord[k+2],cord[k+3]))
    time.sleep(2)
    ptuclient.send_goal_and_wait(ptubwrd)
    speak.text = speakarray[2]   #ask them to move behind robot             
    maryclient.send_goal_and_wait(speak)                                                                                                                
    k = k +4 #move index to next set of cords
    print"moveing to waypoint"
    while not rospy.is_shutdown():                            
        if end == False:        
            if robotstate <3:
                rotatehead(1)
                speak.text = speakarray[random.randint(3,7)]                
                maryclient.send_goal_and_wait(speak) 
                print("moving to goal")
                if seekinghuman:
                    baseClient.send_goal(makegoal(cord[k],cord[k+1],
                                                  cord[k+2],cord[k+3]))
                    out = 0
                    timer1 = time.time()
                    while robotstate < 3 and out == 0 and not rospy.is_shutdown():
                        robotstate = baseClient.get_state()
                        sqrt = 0.0
                        if robotstate > 2:   
                            print "in seeking human, current robot state = {}".format(robotstate)
                            time.sleep(2)
                            
                        rospy.Subscriber("/people_tracker/pose", PoseStamped, wherehuman)
                        if (timer1+robotquickWaitTime) < time.time():
                            out = 1
                            baseClient.cancel_all_goals()
                            print "could not see a human following, will stop and wait"
                            speak.text = "could not see a human following, will stop and wait"                
                            maryclient.send_goal_and_wait(speak) 
                            robotstate = 3
                            #~~~~~
                            dlist.append(' not spotted huamen {} times in seek'.format(countseek))
                            #dlist.append("{:.2f} seconds it took to spot human".format(timer - time.time()))
                            countseek = countseek +1
                            #~~~~~                             
                        elif sqrt >humanMinDistance and sqrt <= humanMaxDistance:
                            timer1 = time.time()
                            #~~~~~
                            #dlist.append('human spotted at {} in seek'.format(sqrt))                            
                            sqrt = 0.0
                            
                        else:                            
                            print "seeking human for {:.2f} seconds ".format(((timer1+robotquickWaitTime) - time.time()))
                            
                else:
                    print "in else so not seeking human"
                    baseClient.send_goal_and_wait(makegoal(cord[k],cord[k+1],
                                                       cord[k+2],cord[k+3]))    #send goal cords
                    robotstate = baseClient.get_state()#get the current state of robot
                 
            if robotstate == 3:
                if out < 1:
                    print'goal reached, waiting for human. \nCurrently at waypoint {}/{}'.format(((k+4)/4),(len(cord)/4))
                    speak.text = speakarray[random.randint(8,10)]                
                    maryclient.send_goal_and_wait(speak)

                sqrt = 0.0
                waitForHuman = True
                timer = time.time()
                robotstate=0
                speak.text = speakarray[random.randint(11,13)]                
                maryclient.send_goal_and_wait(speak) 
                rotatehead(0)                
                while waitForHuman and not rospy.is_shutdown():                  
                    #print "waiting for {:.2f} seconds ".format(((timer+robotWaitTime) - time.time()))
                    rospy.Subscriber("/people_tracker/pose", PoseStamped, wherehuman)                                                           
                    if sqrt >humanMinDistance and sqrt <= humanMaxDistance:
                        print 'human spotted at {}'.format(sqrt)
                        #~~~~~
                        dlist.append('human spotted at {}'.format(sqrt))
                        dlist.append('spotted huamen {} times'.format(count))
                        dlist.append("{:.2f} seconds it took to spot human".format(timer - time.time()))
                        count = count +1
                        #~~~~~                        
                        if len(cord)-1 > k+4: 
                           sqrt = 0.0
                           speak.text = speakarray[random.randint(18,21)]                
                           maryclient.send_goal_and_wait(speak)
                           if out < 1:
                               k = k +4 #move index to next set of cords
                           out = 0
                           print'next goal coords: X:{} Y:{} Z:{} W:{}'.format(cord[k],cord[k+1],cord[k+2],cord[k+3])
                           print'moving to waypoint {}/{}'.format(((k+4)/4),(len(cord)/4))
                           seekinghuman = True
                        else:                            
                            print'the guiding has finished'
                            end = True
                            seekinghuman = False
                        waitForHuman = False
                        
                    elif (timer+robotWaitTime) < time.time():
                         print "human has not appered after {} seconds".format(robotWaitTime)
                         #~~~~~
                         dlist.append("robot has failed to see human {} times".format(countf))
                         countf = countf +1
                        #~~~~~
                         speak.text = speakarray[random.randint(22,24)]                
                         maryclient.send_goal_and_wait(speak)
                         if  k-4 >= 0:
                             print"Robot will go back to prior waypoint and wait"
                             k = k -4 #move index to prior set of cords
                             out = 1
                             print'next goal coords: X:{} Y:{} Z:{} W:{}'.format(cord[k],cord[k+1],cord[k+2],cord[k+3])
                             print'moving back to prior waypoint {}/{}'.format(((k+4)/4),(len(cord)/4))
                             speak.text = speakarray[random.randint(25,26)]                
                             maryclient.send_goal_and_wait(speak) 
                         else:
                             speak.text = speakarray[27]                
                             maryclient.send_goal_and_wait(speak)                              
                             print 'no human is following robot, will go back to start' 
                             end = True
                         seekinghuman = False
                         waitForHuman = False
                    else:
                        #sqrt = 0.0 # for debug-remove and uncommet below once done
                        print 'waiting for human'               
                                      
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
                with open(os.path.dirname(os.path.realpath(__file__)) + '/Method3_stats_stuck.txt','a') as File:
                    for item in dlist:
                        print>>File, item                 
                #~~~~~
        if end:
            rotatehead(1)
            if robotstate <3:
                print'moving robot back to start'
                speak.text = speakarray[random.randint(14,15)]                
                maryclient.send_goal_and_wait(speak)
                speak.text = speakarray[16]                
                maryclient.send_goal_and_wait(speak)
                baseClient.send_goal_and_wait(makegoal(-2.0,0.0,1.0,0.1))
                robotstate = baseClient.get_state() #get the current state of robot
                
            if robotstate >=3:
                 ptuclient.send_goal_and_wait(ptuforward)
                 speak.text = speakarray[17]
                 maryclient.send_goal_and_wait(speak)
                 #~~~~~
                 dlist.append('guide end, took {:.2f} seconds'.format(startimer - time.time()))
                 dlist.append('guide end, took {:.2f} seconds'.format((startimer - time.time())/60))                 
                 dlist.append('=========== end ======== \n')
                 with open(os.path.dirname(os.path.realpath(__file__)) + '/Method3_stats.txt','a') as File:
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
