#!/usr/bin/env python
"""Explicitly specify goals to get a simple navigation and manipulation demo.

"""
from __future__ import with_statement # for python 2.5

import time
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def getGraspLoc(obj):
    """gets the grasp location on a table object"""
    tableextents = obj.ComputeAABB().extents()
    tableloc = obj.GetConfigurationValues()
    return [tableloc[0] + tableextents[0], tableloc[1], tableloc[2]] 
    
def getRobotGoal(obj, left=True):
    """gets the robot location left of the goal object"""
    tableextents = obj.ComputeAABB().extents()
    tableloc = obj.GetConfigurationValues()
    robotbuffer = 0.6
    mult = -1 if left else 1
    return [tableloc[0]+ mult*tableextents[0] + mult*robotbuffer, tableloc[1], 0, 0 if left else 3.14]

#def grabTable(self, table):
#	"""Moves the arm and manipulator to grasp a table"""
#	openhandfn = lambda: self.MoveToPosition([-0.7],self.robot.GetActiveManipulator().GetGripperIndices())
#	TTable = disk.GetTransform()
#	for ang2 in arange(-pi/2,1.5*pi,0.4):
#		for ang1 in arange(-0.6,0,0.2):
#			Tgrasps = self.GetGrasp(TTable, disk.radius, [ang1,ang2]) # get the grasp transform given the two angles
#			for Tgrasp in Tgrasps: # for each of the grasps
#				try:
#					raveLogInfo('opening hand')
#					openhandfn()
#					raveLogInfo('moving hand to location')
#					self.basemanip.MoveToHandPosition(matrices=[Tgrasp],maxtries=1)
#					self.waitrobot()
#					raveLogInfo('succeeded so grab the table')
#					self.taskmanip.CloseFingers()
#					self.waitrobot()
#					with self.env:
#						self.robot.Grab(table)
#
#					return True
#				except planning_error,e:
#					raveLogWarn(str(e))
#					with self.env:
#						self.robot.ReleaseAllGrabbed()
#						table.Enable(False)
#					openhandfn()
#					with self.env:
#						table.Enable(True)
#	return False


def main(env,options):
    "Main example code."
    # load the environment XML file
    env.Load('twopr2.env.xml')
    time.sleep(1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot1'
    # 3) get the 2nd robot that is inside the loaded scene
    # 4) assign it to the variable named 'robot2'
    robot1 = env.GetRobots()[0]
    robot2 = env.GetRobots()[1]

    manip1 = robot1.SetActiveManipulator('leftarm_torso') # set the manipulator to leftarm + torso
    ikmodel1 = databases.inversekinematics.InverseKinematicsModel(robot1,iktype=IkParameterization.Type.Transform6D)
    if not ikmodel1.load():
        ikmodel1.autogenerate()

    manip2 = robot2.SetActiveManipulator('leftarm_torso') # set the manipulator to leftarm + torso
    ikmodel2 = databases.inversekinematics.InverseKinematicsModel(robot2,iktype=IkParameterization.Type.Transform6D)
    if not ikmodel2.load():
        ikmodel2.autogenerate()

    # create the interface for basic manipulation programs
    basemanip1 = interfaces.BaseManipulation(robot1,plannername=options.planner)
    taskprob1 = interfaces.TaskManipulation(robot1,plannername=options.planner)
    basemanip2 = interfaces.BaseManipulation(robot2,plannername=options.planner)
    taskprob2 = interfaces.TaskManipulation(robot2,plannername=options.planner)

    # get the table object
    table = env.GetKinBody('Table')
    mug = env.GetKinBody('Mug')

#    # moves the robots' arms down towards the body
#    with env:
#        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
#        goal = [1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]
#	robot1.SetActiveDOFs([robot1.GetJoint(name).GetDOFIndex() for name in jointnames])
#        robot2.SetActiveDOFs([robot2.GetJoint(name).GetDOFIndex() for name in jointnames])
#        basemanip1.MoveActiveJoints(goal=goal)
#        basemanip2.MoveActiveJoints(goal=goal)
#    waitrobot(robot2)
    
    # move robot to the goal location (navigate using the mobile base)
    # TODO: need to figure out the goal location for each robot (sides of the table)
    print 'move robot1 base to target'
    with env:
	robot1.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.Z|DOFAffine.RotationAxis,[0,0,1])
    	basemanip1.MoveActiveJoints(goal=getRobotGoal(table, True),maxiter=5000,steplength=0.15,maxtries=2)
    waitrobot(robot1)
    taskprob1.ReleaseFingers()
    waitrobot(robot1)

    print 'move robot2 base to target'
    with env:
	robot2.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.Z|DOFAffine.RotationAxis,[0,0,1])
    	basemanip2.MoveActiveJoints(goal=getRobotGoal(table, False),maxiter=5000,steplength=0.15,maxtries=2)
    waitrobot(robot2)
    taskprob2.ReleaseFingers()
    waitrobot(robot2)

    # move the robots' arm to the table
    print 'move robot1 arm to the target'

    ikparam = IkParameterization([localtarget,t],IkParameterization.Type.TranslationLocalGlobal6D) 
    basemanip.MoveToHandPosition(ikparam=ikparam) 
    
    b = getGraspLoc(table)
    robot1_Tgoal1 = array([[1,0,0,b[0]],[0,1,0,b[1]],[0,0,1,b[2]],[0,0,0,1]])
    basemanip1.MoveToHandPosition(translation=b,seedik=16)
    waitrobot(robot2)
    # set the right arm as active manipulator and move it
    #robot1.SetActiveManipulator('rightarm_torso') # set the manipulator to rightarm + torso
    #basemanip1 = interfaces.BaseManipulation(robot1,plannername=options.planner)
    #basemanip1.MoveToHandPosition(matrices=[robot1_Tgoal2],seedik=16)
    #waitrobot(robot1)
    
    print 'move robot2 arm to the target'
    robot2_Tgoal1 = array([[0,-1,0,3.5],[-1,0,0,-1.3],[0,0,-1,0.842],[0,0,0,1]])
    robot2_Tgoal2 = array([[0,-1,0,3.5],[-1,0,0,-1.3],[0,0,-1,0.842],[0,0,0,1]])
    #basemanip2.MoveToHandPosition(matrices=[robot2_Tgoal1],seedik=16)
    waitrobot(robot2)
    # set the right arm as active manipulator and move it
    #robot2.SetActiveManipulator('rightarm_torso') # set the manipulator to rightarm + torso
    #basemanip2 = interfaces.BaseManipulation(robot2,plannername=options.planner)
    #basemanip2.MoveToHandPosition(matrices=[robot2_Tgoal2],seedik=16)
    #waitrobot(robot2)

    print 'close fingers until collision'
    taskprob1.CloseFingers()
    waitrobot(robot1)

    target=env.GetKinBody('Table')
    print 'move the arm with the target back to the initial position'
    with env:
        robot1.Grab(target)
        basemanip1.MoveManipulator(goal=[0, 0, 1.29023451, 0, -2.32099996, 0, -0.69800004, 0])
    waitrobot(robot1)

    print 'move the robot to another location'
    with env:
        robot1.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])
        localgoal = [0,2.4,0]
        T = robot1.GetTransform()
        goal = dot(T[0:3,0:3],localgoal) + T[0:3,3]
        with robot1:
            robot1.SetActiveDOFValues(goal)
            incollision = env.CheckCollision(robot1)
            if incollision:
                print 'goal in collision!!'

    basemanip1.MoveActiveJoints(goal=goal,maxiter=5000,steplength=0.15,maxtries=2)
    waitrobot(robot1)

#
#    print 'move the arm to the designated position on another table to place the target down'
#    Tgoal = array([[0,-1,0,3.5],[-1,0,0,1.5],[0,0,-1,0.855],[0,0,0,1]])
#    res = basemanip.MoveToHandPosition(matrices=[Tgoal],seedik=16)
#    waitrobot(robot)
#
#    taskprob.ReleaseFingers(target=target)
#    waitrobot(robot)
#
#    print 'move manipulator to initial position'
#    basemanip.MoveManipulator(goal=[0, 0, 1.29023451, 0, -2.32099996, 0, -0.69800004, 0])
#    waitrobot(robot)
#
#    print 'close fingers until collision'
#    taskprob.CloseFingers()
#    waitrobot(robot)

from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments

@openravepy.with_destroy
def run(args=None):
    """Command-line execution of the example.

    :param args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description='Explicitly specify goals to get a simple navigation and manipulation demo.', usage='openrave.py --example simplemanipulation [options]')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--planner',action="store",type='string',dest='planner',default=None,
                      help='the planner to use')
    (options, leftargs) = parser.parse_args(args=args)
    OpenRAVEGlobalArguments.parseAndCreateThreadedUser(options,main,defaultviewer=True)


if __name__ == "__main__":
    run()
