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

def getRobotGoal(obj, left=True):
    """gets the robot location left of the goal object"""
    tableextents = obj.ComputeAABB().extents()
    tableloc = obj.GetConfigurationValues()
    robotxbuffer = 0.855
    robotybuffer = 0.116
    mult = -1 if left else 1
    return [tableloc[0]+ mult*tableextents[0] + mult*robotxbuffer, tableloc[1] + robotybuffer, 0, 0 if left else 3.14]

def main(env,options):
    # load the environment XML file
    env.Load('twopr2.env.xml')
    time.sleep(1)

    # get the two robots from the environment
    robot1 = env.GetRobots()[0]
    robot2 = env.GetRobots()[1]

    manip1 = robot1.SetActiveManipulator('rightarm_torso') # set robot1's manipulator to leftarm + torso
    ikmodel1 = databases.inversekinematics.InverseKinematicsModel(robot1,iktype=IkParameterization.Type.Transform6D)
    if not ikmodel1.load():
        ikmodel1.autogenerate()

    manip2 = robot2.SetActiveManipulator('leftarm_torso') # set robot2's manipulator to leftarm + torso
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

    # move robot to the goal location (navigate using the mobile base)
    print 'move robots to target'
    with env:
        robot1.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.Z|DOFAffine.RotationAxis,[0,0,1])
        basemanip1.MoveActiveJoints(goal=getRobotGoal(table, True),maxiter=5000,steplength=0.15,maxtries=2)
        robot2.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.Z|DOFAffine.RotationAxis,[0,0,1])
        basemanip2.MoveActiveJoints(goal=getRobotGoal(table, False),maxiter=5000,steplength=0.15,maxtries=2)
    waitrobot(robot2)

    # moves the robots' arms down towards the body
    print 'moving arms'
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        goal = [1.29023451,-2.32099996,0.0,1.27843491,-2.32100002,0.0]
        robot1.SetActiveDOFs([robot1.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot2.SetActiveDOFs([robot2.GetJoint(name).GetDOFIndex() for name in jointnames])
        basemanip1.MoveActiveJoints(goal=goal)
        basemanip2.MoveActiveJoints(goal=goal)
    waitrobot(robot2)

    print 'releasing fingers'
    taskprob1.ReleaseFingers()
    taskprob2.ReleaseFingers()
    waitrobot(robot2)

    print 'move the arms to the target'
    Tgoal2 = array([[1,0,0,0.8],[0,1,0,-0.14],[0,0,1,0.91],[0,0,0,1]])
    res = basemanip2.MoveToHandPosition(matrices=[Tgoal2],seedik=16)
    Tgoal1 = array([[1,0,0,-0.8],[0,1,0,-0.14],[0,0,1,0.91],[0,0,0,1]])
    res = basemanip1.MoveToHandPosition(matrices=[Tgoal1],seedik=16)
    waitrobot(robot2)

    print 'orienting hands'
    with env:
        robot2.SetActiveDOFs([robot2.GetJoint('l_wrist_flex_joint').GetDOFIndex(), robot2.GetJoint('l_wrist_roll_joint').GetDOFIndex()])
        basemanip2.MoveActiveJoints(goal=[0.2, 1.1])
        robot1.SetActiveDOFs([robot1.GetJoint('r_wrist_flex_joint').GetDOFIndex(), robot1.GetJoint('r_wrist_roll_joint').GetDOFIndex()])
        basemanip1.MoveActiveJoints(goal=[0.2, 1.1])
    waitrobot(robot2)

    print 'move closer'
    with env:
        robot2.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.Z,[0,0,1])
        basemanip2.MoveActiveJoints(goal=[1.411,0.116,0.05],maxiter=5000,steplength=0.15,maxtries=2)
        robot1.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.Z,[0,0,1])
        basemanip1.MoveActiveJoints(goal=[-1.411,0.116,0.05],maxiter=5000,steplength=0.15,maxtries=2)
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
        localgoal = [0, 0.2, math.pi]
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
