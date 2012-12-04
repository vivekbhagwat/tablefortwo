#!/usr/bin/env python
"""Explicitly specify goals to get a simple navigation and manipulation demo.

"""
from __future__ import with_statement # for python 2.5

import math
import openravepy
import time
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def getGoalCoordsNearObj(obj, left=True):
    """gets the robot location near the goal object"""
    tableextents = obj.ComputeAABB().extents()
    tableloc = obj.GetConfigurationValues()
    robotxbuffer = 0.855
    robotybuffer = 0.116
    mult = -1 if left else 1
    xloc = tableloc[0]+ mult*tableextents[0] + mult*robotxbuffer
    yloc = tableloc[1] + robotybuffer
    return [xloc, yloc, 0, 0 if left else math.pi]

def getGraspLoc(obj, left=True):
    """ gets the pre-grasp location near the object """
    tableextents = obj.ComputeAABB().extents()
    tableloc = obj.GetConfigurationValues()
    xbuf = 0.20
    ybuf = 0.16
    zbuf = 0.16
    mult = -1 if left else 1
    xloc = tableloc[0] + mult*tableextents[0] + mult*xbuf
    yloc = tableloc[1] - tableextents[1] + ybuf
    zloc = tableloc[2] + zbuf
    return [xloc, yloc, zloc]    

def getFinalGraspPos(robot, left=True):
    robotpos = robot.GetTransform()[0:3,3]
    mult = 1 if left else -1
    buf = 0.044
    return [robotpos[0] + mult*buf, robotpos[1], robotpos[2], 0 if left else math.pi]

def foldUpArms(robot, basemanip):
    """ moves the robot's arms in towards the body """
    jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint',
                  'r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
    goal = [1.29023451, -2.32099996, 0.0, 1.27843491, -2.32100002, 0.0]
    robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
    return basemanip.MoveActiveJoints(goal=goal)

def navigateToGoal(robot, basemanip, goal):
    """ moves the robot's base to a specified goal [x,y,z,rotation] """
    robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.Z|DOFAffine.RotationAxis,[0,0,1])
    return basemanip.MoveActiveJoints(goal=goal,maxiter=5000,steplength=0.15,maxtries=2)

def reachToPosition(basemanip, goal):
    """ reach out the robot's active manipulator to a goal [x,y,z] in front of the robot """
    Tgoal = array([[1,0,0,goal[0]],[0,1,0,goal[1]],[0,0,1,goal[2]],[0,0,0,1]])
    return basemanip.MoveToHandPosition(matrices=[Tgoal],seedik=16)

def moveJointToValue(robot, basemanip, joint, value):
    """ move a joint (specified by a string) to a specific joint value """
    robot.SetActiveDOFs([robot.GetJoint(joint).GetDOFIndex()])
    return basemanip.MoveActiveJoints(goal=[value])

def main(env, options):
    # load the environment XML file
    env.Load(options.environment)
    time.sleep(1)

    # get the two robots from the environment
    robot1 = env.GetRobots()[0]
    robot2 = env.GetRobots()[1]

    # set the robots' active manipulators
    manip1 = robot1.SetActiveManipulator('rightarm_torso')
    manip2 = robot2.SetActiveManipulator('leftarm_torso')

    # create the interface for basic manipulation programs
    basemanip1 = interfaces.BaseManipulation(robot1,plannername=options.planner)
    basemanip2 = interfaces.BaseManipulation(robot2,plannername=options.planner)
    taskprob1 = interfaces.TaskManipulation(robot1,plannername=options.planner)
    taskprob2 = interfaces.TaskManipulation(robot2,plannername=options.planner)

    # get the table object
    table = env.GetKinBody('Table')

    # move robot to the goal location (navigate using the mobile base)
    print 'move robots to target'
    with env:
        print getGoalCoordsNearObj(table, True)
        print getGoalCoordsNearObj(table, False)
        navigateToGoal(robot1, basemanip1, getGoalCoordsNearObj(table, True))
        navigateToGoal(robot2, basemanip2, getGoalCoordsNearObj(table, False))
    waitrobot(robot2)

    # move robots' arms in towards their bodies
    print 'moving arms'
    with env:
        foldUpArms(robot1, basemanip1)
        foldUpArms(robot2, basemanip2)
    waitrobot(robot2)

    print 'releasing fingers'
    taskprob1.ReleaseFingers()
    taskprob2.ReleaseFingers()
    waitrobot(robot2)

    print 'move the arms to the target'
    reachToPosition(basemanip1, getGraspLoc(table, True))
    reachToPosition(basemanip2, getGraspLoc(table, False))
    waitrobot(robot2)

    print 'orienting hands'
    with env:
        moveJointToValue(robot1, basemanip1, 'r_wrist_flex_joint', 0.2)
        moveJointToValue(robot2, basemanip2, 'l_wrist_flex_joint', 0.2)
    waitrobot(robot2)
    with env:
        moveJointToValue(robot1, basemanip1, 'r_wrist_roll_joint', 1.1)
        moveJointToValue(robot2, basemanip2, 'l_wrist_roll_joint', 1.1)
    waitrobot(robot2)

    print 'move closer'
    with env:
        navigateToGoal(robot1, basemanip1, getFinalGraspPos(robot1, True))
        navigateToGoal(robot2, basemanip2, getFinalGraspPos(robot2, False))
    waitrobot(robot2)

    print 'close fingers until collision'
    taskprob2.CloseFingers()
    taskprob1.CloseFingers()
    waitrobot(robot2)

    target=env.GetKinBody('Table')
    print 'grab target and robot1!'
    with env:
        robot2.Grab(target)
        robot2.Grab(robot1)
    waitrobot(robot2)

    print 'try to move everything'
    with env:
        robot2.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])
        localgoal = [0, 1.0, math.pi]
        T = robot2.GetTransform()
        goal = dot(T[0:3,0:3],localgoal) + T[0:3,3]
        basemanip2.MoveActiveJoints(goal=goal,maxiter=10,steplength=0.50,maxtries=2)
    waitrobot(robot2)

    while True:
        try:
             raw_input('Press Ctrl-D to exit')
        except EOFError:
             break

from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments

@openravepy.with_destroy
def run(args=None):
    """Command-line execution of the two robot manipulation.
    """
    parser = OptionParser(description='Have two robots navigate their environment with a table.', usage='python grasp_stuff.py [options]')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--planner',action="store",type='string',dest='planner',default=None,
                      help='the planner to use')
    parser.add_option('--environment',action="store",type='string',dest='environment',default='twopr2.env.xml',
                      help='the environment to use')
    (options, leftargs) = parser.parse_args(args=args)
    OpenRAVEGlobalArguments.parseAndCreateThreadedUser(options,main,defaultviewer=True)

if __name__ == "__main__":
    run()
