#!/usr/bin/env python3


import asyncio
import sys
import cozmo

from frame2d import Frame2D 


robotFrames = []
cubeFrames = []
cliffSensor = []

if len(sys.argv) == 2:
	logName = sys.argv[1]+".py"
else:
	logName = "10cmcube 30deg.py"


async def cozmo_program(robot: cozmo.robot.Robot):
	for t in range(100):
		#print("Robot pose: " + str(robot.pose))
		robotPose = Frame2D.fromPose(robot.pose)
		#print("Robot pose 2D frame: " + str(robotPose))
		robotFrames.append((t,robotPose))

		cliffSensor.append((t,robot.is_cliff_detected))

		cubeIDs = (cozmo.objects.LightCube1Id,cozmo.objects.LightCube2Id,cozmo.objects.LightCube3Id)
		for cubeID in cubeIDs:
			cube = robot.world.get_light_cube(cubeID)
			if cube is not None and cube.is_visible:
				#print("Visible: " + cube.descriptive_name + " (id=" + str(cube.object_id) + ")")
				#print("   pose: " + str(cube.pose))
				cubePose2D = Frame2D.fromPose(cube.pose)
				#print("   2D frame: " + str(cubePose2D))
				cubeFrames.append((t,cubePose2D))
		#print()
		await asyncio.sleep(0.1)

	logFile = open(logName, 'w')

	print("from frame2d import Frame2D", file=logFile)
	print("robotFrames = [", file=logFile)
	for idx in range(len(robotFrames)):
		t = robotFrames[idx][0]
		x = robotFrames[idx][1].x()
		y = robotFrames[idx][1].y()
		a = robotFrames[idx][1].angle()
		print("   (%d, Frame2D.fromXYA(%f,%f,%f))" % (t,x,y,a), end="", file=logFile)
		if idx != len(robotFrames)-1:
			print(",", file=logFile)
	print("]", file=logFile)

	print("cubeFrames = [", file=logFile)
	for idx in range(len(cubeFrames)):
		t = cubeFrames[idx][0]
		x = cubeFrames[idx][1].x()
		y = cubeFrames[idx][1].y()
		a = cubeFrames[idx][1].angle()
		print("   (%d, Frame2D.fromXYA(%f,%f,%f))" % (t,x,y,a), end="", file=logFile)
		if idx != len(cubeFrames)-1:
			print(",", file=logFile)
	print("]", file=logFile)
	print("cliffSensor = [", file=logFile)
	for idx in range(len(cliffSensor)):
		t = cliffSensor[idx][0]
		s = cliffSensor[idx][1]
		print("   ("+str(t)+", "+str(s)+")", end="", file=logFile)
		if idx != len(cliffSensor)-1:
			print(",", file=logFile)
	print("]", file=logFile)


cozmo.robot.Robot.drive_off_charger_on_connect = False
cozmo.run_program(cozmo_program, use_3d_viewer=True, use_viewer=False)



