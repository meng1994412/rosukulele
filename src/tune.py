#! /usr/bin/env python
import rospy
from rosukulele.srv import *
import intera_interface
head_display = intera_interface.HeadDisplay()

OFFSET_ABSOLUTE = [0.000,0.015,0]

NEUTRAL_LOC = [0,-0.25,0.4]
ORIENTATIONS = ['0 1 0 0', '-0.5 -0.5 0.5 -0.5'] #0 down, 1 sideways

STRING_APPROACH = [-0.046,-0.366,0.2]
STRING_LOC = [[[-0.046, -0.375, 0.1343],[-0.02,-0.41,0.1343]],\
	[[-0.02,-0.3614,0.1465],[-0.02,-0.385,0.1465]],\
	[[-0.02,-0.3485,0.1465],[-0.02,-0.376,0.1465]],\
	[[-0.02,-0.333,0.1465],[-0.02,-0.359,0.1465]]]

PICK_APPROACH = [-0.1,0.035,0.43]
PICK_LOC = [-0.1,0.046,0.21]

TUNER_APPROACH = [-0.1,-0.05,0.43]
TUNER_LOC = [-0.1,-0.04,0.205]

HEAD_LOC = [0.25,-0.36,0.3]

PEG_APPROACH = [[0.248,-0.435,0.06],[1,1,1],[2,2,2],[3,3,3],[4,4,4]]
PEG_LOC = [[0.248,-0.413,0.06],[1,1,1],[2,2,2],[3,3,3],[4,4,4]]

STRING_NAME = ['A', 'E', 'C', 'G']
PITCHES = [69, 64, 60, 67]
PITCH_TOL = 0.15

currentOrientation = -1 #not an official orientation
pegAngles = [0.,0.,0.,0.]

def main():

	rospy.init_node('tune')
	
	#move to an absolute location
	#TODO

	# for currentString in range(0,1):
	# 	#check initial pitch
	# 	toPick('c')
	# 	error = pluck(currentString)
	# 	toPick('o')

	# 	while (error > PITCH_TOL):
	# 		#correct pitch
	# 		toTuner('c')
	# 		tune(currentString, error)
	# 		toTuner('o')

	# 		#check new pitch
	# 		toPick('c')
	# 		error = pluck(currentString)
	# 		toPick('o')

	# 	print("The "+STRING_NAME[currentString]+" String is in tune!")
	#*************************************Grip blocks*************************************
	# moveLoc(NEUTRAL_LOC)
	# setOrientation(0)
	# toPick('c')
	# toPick("o")
	# moveLoc(NEUTRAL_LOC)
	# toTuner("c")
	# toTuner("o")
	# **************************************************************************************
	tuned = False
	angle = 0
	moveLoc(NEUTRAL_LOC)
	grip('o')
	while not tuned:
		toPick('c')
		print("Note Error")
		note_err = pluck(0)
		print(note_err)
		toPick('o')
		if note_err < 0.15 and note_err > -0.15:
			tuned = True
		else:
			toTuner('c')
			angle = tune(0,1,(angle))
			toTuner('o')
	head_display.display_image("/home/levi/sawyerws/src/rosukulele/Images/Israel2.png")


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

	head_display.display_image("/home/levi/sawyerws/src/rosukulele/Images/Israel1.png")
	setOrientation(1)
	moveLoc(STRING_APPROACH)
	moveLoc(STRING_LOC[currentString][0])
	moveLoc(STRING_LOC[currentString][1])
	pitch = getPitch(0)
	error = pitch - PITCHES[currentString]
	moveLoc(STRING_APPROACH)
	moveLoc(NEUTRAL_LOC)
	return error

def tune(currentString, error,angle):
	global pegAngles
	PGAIN = 1.6
	head_display.display_image("/home/levi/sawyerws/src/rosukulele/Images/Israel3.png")
	setOrientation(1)
	moveLoc(HEAD_LOC)
	moveLoc(PEG_APPROACH[currentString])
	#rotate(pegAngle[currentString])
	rotate(angle)
	moveLoc(PEG_LOC[currentString])
	rad = PGAIN*error; #should really be some other function of error
	rotate(rad)
	#pegAngle[currentString]+=rad
	moveLoc(PEG_APPROACH[currentString])
	rotate(-angle)
	rotate(-rad)
	moveLoc(HEAD_LOC)
	moveLoc(NEUTRAL_LOC)
	return rad

def grip(state):
	#closes or opens the gripper. 'o' opens and 'c' closes
	rospy.wait_for_service('grip_pls')
	try:
		grip_handle=rospy.ServiceProxy('grip_pls', Grip)
		reply = grip_handle(state)
		print reply.success
		return 
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def moveLoc(locArray):
	#Moves to a location defined by a 3 length array in the Uke frame
	# locArray[0]+=OFFSET_ABSOLUTE[0]
	# locArray[1]+=OFFSET_ABSOLUTE[1]
	# locArray[2]+=OFFSET_ABSOLUTE[2]

	locString = ukeToBase(locArray)
	msg = '-p '+ locString;
	rospy.wait_for_service('move_to')
	try:
		grip_handle=rospy.ServiceProxy('move_to', MoveTo)
		reply = grip_handle(msg)
		print reply.response
		return
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
			print reply.response
			return
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
	
def rotate(rad):
	#A Positive rotation tunes up
	global currentOrientation
	currentOrientation = -1

	rotation = '-T -R 0 0 0 0 0 '+str(rad)
	rospy.wait_for_service('move_to')
	try:
		grip_handle=rospy.ServiceProxy('move_to', MoveTo)
		reply = grip_handle(rotation)
		print reply.response
		return
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