#! /usr/bin/env python
import rospy
from rosukukulele.srv import *

NEUTRAL_LOC = [0,0,0]

STRING_APPROACH = [0,0,0]
STRING_LOC = [[0,0,0],[1,1,1],[2,2,2],[3,3,3],[4,4,4]]

PICK_APPROACH = [0,0,0]
PICK_LOC = [0,0,0]

TUNER_APPROACH = [0,0,0]
TUNER_LOC = [0,0,0] 

PITCHES = [67, 60, 64, 69] #G, C, E, A


def main():

    rospy.init_node('tune')

    
    currentString = 0;
    #set to a known set of Joint Angles
    ##TODO
    toPick('c')
    pluck(currentString)
    toPick('o')
    toTuner('c')
    #tune
    toTuner('o')
    #something involving listening and getting a pitch
    
    moveLoc(STRING_APPROACH)
    
    

def toPick(state):
    #move to and grab/set down pick. 'o' opens and 'c' closes
    moveLoc(PICK_APPROACH)
    moveLOC(PICK_LOC)
    grip(state)
    moveLoc(PICK_APPROACH)
    moveLoc(NEUTRAL_LOC)

def toTuner(state):
    moveLoc(TUNER_APPROACH)
    moveLOC(TUNER_LOC)
    grip(state)
    moveLoc(TUNER_APPROACH)
    moveLoc(NEUTRAL_LOC)
    


def pluck(currentString):
	moveLoc(STRING_APPROACH)
    moveLoc(STRING_LOC[currentString])
    moveLoc(SRING_LOC[currentString+1])
    #Listen
    moveLoc(STRING_APPROACH)
    moveLoc(NEUTRAL_LOC)
    #return 

def grip(state):
	#closes or opens the gripper. 'o' opens and 'c' closes
    rospy.wait_for_service('grip_pls')
    try:
        grip_handle=rospy.ServiceProxy('grip_pls', Grip)
        reply = grip_handle(state)
        return reply.success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e



def moveLoc(locArray):
	#Moves to a location defined by a 3 length array
	locString = '-p '+' '.join(str(e) for e in locArray)
    rospy.wait_for_service('move_to')
    try:
        grip_handle=rospy.ServiceProxy('move_to', MoveTo)
        reply = grip_handle(locString)
        return reply.response
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    

if __name__ == '__main__':
    main()