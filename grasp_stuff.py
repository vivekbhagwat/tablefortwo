#!/usr/bin/env python
"""Explicitly specify goals to get a simple navigation and manipulation demo.

"""
from __future__ import with_statement # for python 2.5

import time
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

class FastGrasping:
    class GraspingException(Exception):
        def __init__(self,args):
            self.args=args

    def __init__(self,robot,target):
        self.robot = robot
        self.ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.Transform6D)
        if not self.ikmodel.load():
            self.ikmodel.autogenerate()
        self.gmodel = databases.grasping.GraspingModel(robot,target)
        self.gmodel.init(friction=0.4,avoidlinks=[])

    def checkgraspfn(self, contacts,finalconfig,grasp,info):
        # check if grasp can be reached by robot
        Tglobalgrasp = self.gmodel.getGlobalGraspTransform(grasp,collisionfree=True)
        # have to set the preshape since the current robot is at the final grasp!
        self.gmodel.setPreshape(grasp)
        sol = self.gmodel.manip.FindIKSolution(Tglobalgrasp,True)
        if sol is not None:
            jointvalues = array(finalconfig[0])
            jointvalues[self.gmodel.manip.GetArmIndices()] = sol
            raise self.GraspingException([grasp,jointvalues])
        return True

    def computeGrasp(self):
        approachrays = self.gmodel.computeBoxApproachRays(delta=0.02,normalanglerange=0.5) # rays to approach object
        standoffs = [0]
        # roll discretization
        rolls = arange(0,2*pi,0.5*pi)
        # initial preshape for robot is the released fingers
        with self.gmodel.target:
            self.gmodel.target.Enable(False)
            taskmanip = interfaces.TaskManipulation(self.robot)
            final,traj = taskmanip.ReleaseFingers(execute=False,outputfinal=True)
            preshapes = array([final])
        try:
            self.gmodel.disableallbodies=False
            self.gmodel.generate(preshapes=preshapes,standoffs=standoffs,rolls=rolls,approachrays=approachrays,checkgraspfn=self.checkgraspfn,graspingnoise=0.01)
            return None,None # did not find anything
        except self.GraspingException, e:
            return e.args


def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

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
    #freeind = robot1.GetActiveManipulator().GetArmIndices()[3:] 
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

    trans = robot2.GetActiveManipulator().GetEndEffectorTransform()
    print trans

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

    print 'move the arm to the target'
    Tgoal = array([[1,0,0,0.5],[0,1,0,0.16],[0,0,1,0.86],[0,0,0,1]])
    res = basemanip2.MoveToHandPosition(matrices=[Tgoal],seedik=16)
    waitrobot(robot2)

    print 'orienting hand'
    with env:
        robot2.SetActiveDOFs([robot2.GetJoint('l_wrist_flex_joint').GetDOFIndex(), robot2.GetJoint('l_wrist_roll_joint').GetDOFIndex()])
        basemanip2.MoveActiveJoints(goal=[0.2, 1.1])
    waitrobot(robot2)
    
    print 'move closer'
    with env:
        robot2.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.Z,[0,0,1])
        basemanip2.MoveActiveJoints(goal=[1.11,0.416,0.05],maxiter=5000,steplength=0.15,maxtries=2)
    waitrobot(robot2)

    print 'close fingers until collision'
    taskprob2.CloseFingers()
    waitrobot(robot2)

    #target=env.GetKinBody('Table')
    #print 'move the arm with the target back to the initial position'
    #with env:
    #    robot2.Grab(target)
    #    basemanip2.MoveManipulator(goal=[0, 0, 1.29023451, 0, -2.32099996, 0, -0.69800004, 0])
    #waitrobot(robot2)

    while True:
        try:
             raw_input('Press any key')
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
