#! /usr/bin/env python
import rospy
from rosukulele.srv import *

NEUTRAL_LOC = [0,0,0]
ORIENTATIONS = ['0 1 0 0', '-0.5 -0.5 0.5 -0.5'] #0 down, 1 sideways

STRING_APPROACH = [0,0,0]
STRING_LOC = [[0,0,0],[1,1,1],[2,2,2],[3,3,3],[4,4,4]]

PICK_APPROACH = [-0.1,0.035,0.43]
PICK_LOC = [-0.1,0.035,0.22]

TUNER_APPROACH = [-0.099,-0.05,0.43]
TUNER_LOC = [-0.099,-0.05,0.22]

HEAD_LOC = [0,0,0]

PEG_APPROACH = [[0,0,0],[1,1,1],[2,2,2],[3,3,3],[4,4,4]]
PEG_LOC = [[0,0,0],[1,1,1],[2,2,2],[3,3,3],[4,4,4]]

STRING_NAME = ['A', 'E', 'C', 'G']
PITCHES = [69, 64, 60, 67]
PITCH_TOL = 0.15

currentOrientation = -1 #not an official orientation
pegAngles = [0.,0.,0.,0.]

def main():

	rospy.init_node('tune')
	
	#move to an absolute location
	#TODO

	for currentString in range(0,1):
		#check initial pitch
		toPick('c')
		error = pluck(currentString)
		toPick('o')

		while (error > PITCH_TOL):
			#correct pitch
			toTuner('c')
			tune(currentString, error)
			toTuner('o')

			#check new pitch
			toPick('c')
			error = pluck(currentString)
			toPick('o')

		print("The "+STRING_NAME[currentString]+" String is in tune!")




def toPick(state):
	#move to and grab/set down pick. 'o' opens and 'c' closes

	setOrientation(0)
	moveLoc(PICK_APPROACH)
	moveLoc(PICK_LOC)
	grip(state)
	moveLoc(PICK_APPROACH)
	moveLoc(NEUTRAL_LOC)

def toTuner(state):
	#move to and grab/set down tuner. 'o' opens and 'c' closes

	setOrientation(0)
	moveLoc(TUNER_APPROACH)
	moveLoc(TUNER_LOC)
	grip(state)
	moveLoc(TUNER_APPROACH)
	moveLoc(NEUTRAL_LOC)

def pluck(currentString):

	setOrientation(1)
	moveLoc(STRING_APPROACH)
	moveLoc(STRING_LOC[currentString])
	moveLoc(SRING_LOC[currentString+1])
	pitch = getPitch(0)
	error = pitch - PITCHES[currentString]
	moveLoc(STRING_APPROACH)
	moveLoc(NEUTRAL_LOC)
	return error

def tune(currentString, error):
	global pegAngles
	PGAIN = 1

	setOrientation(1)
	moveLoc(HEAD_LOC)
	moveLoc(PEG_APPROACH[currentString])
	rotate(pegAngle[currentString])

	moveLoc(PEG_LOC[currentString])
	rad = PGAIN*error; #should really be some other function of error
	rotate(rad)
	pegAngle[currentString]+=rad
	moveLoc(PEG_APPROACH[currentString])
	moveLoc(HEAD_LOC)
	moeLoc(NEUTRAL_LOC)

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
	#Moves to a location defined by a 3 length array in the Uke frame
	locString = ukeToBase(locArray)
	msg = '-p '+ locString;
	rospy.wait_for_service('move_to')
	try:
		grip_handle=rospy.ServiceProxy('move_to', MoveTo)
		reply = grip_handle(msg)
		return reply.response
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def setOrientation(orient):
	global currentOrientation
	if currentOrientation != orient:
		msg = '-o '+ ORIENTATIONS[orient]
		rospy.wait_for_service('move_to')
		try:
			grip_handle=rospy.ServiceProxy('move_to', MoveTo)
			reply = grip_handle(msg)
			currentOrientation = orient
			return reply.response
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
	
def rotate(rad):
	#Moves to a location defined by a 3 length array
	global currentOrientation
	currentOrientation = -1

	rotation = '-R -T 0 0 0 0 0 '+str(rad)
	rospy.wait_for_service('move_to')
	try:
		grip_handle=rospy.ServiceProxy('move_to', MoveTo)
		reply = grip_handle(rotation)
		return reply.response
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def ukeToBase(locArray):
	# Turns an array of coordinates in the ukulele frame 
	# into a string of coordinates in the base frame
	locString = ' '.join(str(e) for e in locArray)
	rospy.wait_for_service('transformation_matrix')
	try:
		grip_handle=rospy.ServiceProxy('transformation_matrix', Transform)
		reply = grip_handle(locString)
		return reply.response
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def getPitch(placeholder):
	rospy.wait_for_service('get_pitch')
	try:
		grip_handle=rospy.ServiceProxy('get_pitch', GetPitch)
		reply = grip_handle(placeholder)
		return reply.pitch
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

if __name__ == '__main__':
	main()