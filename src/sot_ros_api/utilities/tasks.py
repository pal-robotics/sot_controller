from dynamic_graph.sot.core.meta_task_6d import MetaTask6d,toFlags
from dynamic_graph.sot.core.meta_tasks import *
from dynamic_graph.sot.core.meta_tasks_kine import *
from dynamic_graph.sot.core.meta_task_posture import MetaTaskKinePosture
from dynamic_graph.sot.core.meta_task_visual_point import MetaTaskVisualPoint
from dynamic_graph.sot.core.meta_task_grasping_point import MetaTaskGraspingPoint
from dynamic_graph.sot.core.meta_task_velocity_damping import MetaTaskVelocityDamping
from dynamic_graph.sot.core.meta_task_joint_weights import MetaTaskJointWeights
from dynamic_graph.sot.dyninv.task_joint_limit_clamping import TaskJointLimitClamping
from dynamic_graph.sot.core.meta_task_dyn_oppoint_modifier import MetaTaskDynamicOppoint

from sot_ros_api.sot_robot.prologue import robot, solver
from dynamic_graph.sot.dyninv import *

from sot_ros_api.utilities.sot import pop, push

import numpy
import time

class MetaTaskIneqKine6d(MetaTaskKine6d):
    def createTask(self):
        self.task = TaskInequality('inequalitytask'+self.name)
        
    def createFeatures(self):
        self.feature    = FeaturePoint6d('ineqfeature'+self.name)
        self.featureDes = FeaturePoint6d('ineqfeature'+self.name+'_ref')
        self.feature.selec.value = '111111'
        self.feature.frame('current')

def createJointLimitsTask(gain = 1, dt = None):
    robot.dynamic.upperJl.recompute(0)
    robot.dynamic.lowerJl.recompute(0)
    taskJL = TaskJointLimits('jointLimits')
    plug(robot.dynamic.position, taskJL.position)
    """
    The internal gain for the joint limits task is defined as 1/gain, i.e. is used to limit the maximum reachable 
    velocity in a single time interval (dt).
    A high value can be used to limit oscillations around the goal point but it slows down the motion.
    """
    taskJL.controlGain.value = gain
    taskJL.referenceInf.value = robot.dynamic.lowerJl.value
    taskJL.referenceSup.value = robot.dynamic.upperJl.value
    taskJL.selec.value = toFlags(range(6, robot.dimension))
    if(dt):
        taskJL.dt.value = dt
    else:
        plug(robot.device.dt,taskJL.dt)
    return taskJL

def createJointLimitDampingTask(gain= 100, dt= None):
    robot.dynamic.upperJl.recompute(0)
    robot.dynamic.lowerJl.recompute(0)
    robot.dynamic.lowerVl.recompute(0)
    robot.dynamic.upperVl.recompute(0)
    taskJL = TaskJointLimitClamping('jointLimitsClamping')
    plug(robot.dynamic.position, taskJL.position)
    """
    The internal gain for the joint limits task is defined as 1/gain, i.e. is used to limit the maximum reachable 
    velocity in a single time interval (dt).
    A high value can be used to limit oscillations around the goal point but it slows down the motion.
    """
    taskJL.controlGain.value = gain
    taskJL.referenceInf.value = robot.dynamic.lowerJl.value
    taskJL.referenceSup.value = robot.dynamic.upperJl.value
    taskJL.upperVelocityLimits.value = robot.dynamic.upperVl.value
    taskJL.selec.value = toFlags(range(6, robot.dimension))
    if(dt):
        taskJL.dt.value = dt
    else:
        plug(robot.device.dt,taskJL.dt)
    return taskJL

def createEqualityTask(taskName, jointName, gain = None):
    taskEq = MetaTaskKine6d(taskName, robot.dynamic, jointName, jointName)
    taskEq.feature.frame('desired')
    if (gain):
        taskEq.gain.setConstant(gain)
    return taskEq

def createInequalityTask(taskName, jointName, selectionMask='000111', positionVector=(0,0,0), referenceInf=(-100,-100,-100), referenceSup=(100,100,100)):
    taskIneq = MetaTaskIneqKine6d(taskName, robot.dynamic, jointName, jointName)
    taskIneq.feature.frame('desired')
    gotoNd(taskIneq, positionVector, '111')
    taskIneq.feature.selec.value = '111111'
#     taskIneq.task.add(taskIneq.feature.name)
    taskIneq.task.referenceSup.value = referenceSup
    taskIneq.task.referenceInf.value = referenceInf
    taskIneq.task.selec.value = selectionMask
    taskIneq.task.dt.value = 0.001
    taskIneq.task.controlGain.value = 0.9
    return taskIneq

def createVelocityDampingTask(taskName, jointName, collisionCenter, di, ds):
    taskVelDamp = MetaTaskVelocityDamping(taskName, robot.dynamic, jointName, jointName, collisionCenter, di, ds)
    return taskVelDamp
#     taskVelDamp = TaskVelocityDamping(taskName)
#     taskVelDamp.di.value = 0.2
#     taskVelDamp.ds.value = 0.1
#     taskVelDamp.dt.value = 0.001
#     taskVelDamp.controlGain.value = 1
#     
#     taskVelDamp.p2.value = matrixToTuple(goalP2)
#     robot.dynamic.Jarm_right_tool_joint.recompute(0)
#     robot.dynamic.arm_right_tool_joint.recompute(0)
#     plug(robot.dynamic.signal("arm_right_tool_joint"), taskVelDamp.p1)
#     plug(robot.dynamic.signal("Jarm_right_tool_joint"), taskVelDamp.jVel)


def createWeightsTask(diag = None, gain = 0.001, sampleInterval = 0, selec = toFlags(range(0,robot.dimension))):
    taskWeights = MetaTaskJointWeights('jointWeights',selec,robot,diag,gain,sampleInterval)
    plug(robot.device.velocity,taskWeights.task.velocity)
    jointsPos = numpy.zeros(robot.dimension)
    taskWeights.task.setPosition(tuple(jointsPos))
    taskWeights.task.setPositionDes(tuple(jointsPos))
    taskWeights.task.velocity.recompute(0)
    return taskWeights

def createJointsTask(diag = None, gain = 1, selec = toFlags(range(0,robot.dimension)), jointsPos = tuple(numpy.zeros(robot.dimension))):
    taskJoints = MetaTaskJointWeights('jointsTask',selec,robot,diag,gain,0)
    jointsVel = numpy.zeros(robot.dimension)
    taskJoints.task.setVelocity(tuple(jointsVel))
    taskJoints.task.setPositionDes(tuple(jointsPos))
    plug(robot.dynamic.position,taskJoints.task.position)
    return taskJoints

def zeroPosition():
    diag = None
    gain = 0.01
    jointsPos = numpy.zeros(robot.dimension)
    selec = toFlags(range(6,robot.dimension))
    taskZeroPosition = MetaTaskJointWeights('zeroPos',selec,robot,diag,gain,0)
    jointsVel = numpy.zeros(robot.dimension)
    taskZeroPosition.task.setVelocity(tuple(jointsVel))
    taskZeroPosition.task.setPositionDes(tuple(jointsPos))
    plug(robot.dynamic.position,taskZeroPosition.task.position)
    return taskZeroPosition

def safePosition(gain=0.01):
    diag = None
    gain = gain
    safePos = numpy.zeros(robot.dimension)
    safePos[9] = 0.2
    safePos[16] = 0.2
    safePos = (0,0,0,0,0,0,-0.4, 0.4, -0.1, 0.6109, 0, 0, 0, -0.4, 0.4, -0.1, 0.6109, 0, 0, 0, 0.0, 0.0, 0.0, 0.0)
    jointsPos = tuple(safePos)
    selec = toFlags(range(6,robot.dimension))
    taskSafePosition = MetaTaskJointWeights('safePos',selec,robot,diag,gain,0)
    jointsVel = numpy.zeros(robot.dimension)
    taskSafePosition.task.setVelocity(tuple(jointsVel))
    taskSafePosition.task.setPositionDes(tuple(jointsPos))
    plug(robot.dynamic.position,taskSafePosition.task.position)
    return taskSafePosition

def createGazeTask(jointName):
    taskGaze = MetaTaskVisualPoint('gaze'+str(jointName),robot.dynamic, jointName, jointName)
    taskGaze.featureDes.xy.value = (0,0)
    return taskGaze

def createGraspingTask(jointName):
    taskGrasp = MetaTaskGraspingPoint('grasp'+str(jointName), robot.dynamic, jointName, jointName)
    taskGrasp.featureDes.xy.value = (0,0)
    return taskGrasp

def createComEqTask(gain = 1):
    taskCom = MetaTaskKineCom(robot.dynamic)
    robot.dynamic.com.recompute(0)
    taskCom.featureDes.errorIN.value = robot.dynamic.com.value
    taskCom.task.controlGain.value = gain
    return taskCom

def createComIneqTask(gain = 1, dt = 0.001, referenceInf = (-1,-1, 0), referenceSup = (1,1,0)):
    featureCom = FeatureGeneric('featureCom')
    plug(robot.dynamic.com,featureCom.errorIN)
    plug(robot.dynamic.Jcom,featureCom.jacobianIN)
    taskCom = TaskInequality('com')
    taskCom.add(featureCom.name)
    taskCom.selec.value = '011'
    taskCom.referenceInf.value = referenceInf
    taskCom.referenceSup.value = referenceSup
    taskCom.dt.value = dt
    robot.dynamic.com.recompute(0)
    taskCom.controlGain.value = gain
    return taskCom

def gotoNdComp(task,position,selec=None,gain=None,resetJacobian=True,comp=[[0,1,0],[0,0,1],[1,0,0]]):
    '''
    gotoNdComp takes care of the different frame orientations used in jrl-dynamics. 
    Be careful about the comp matrix, it is specific for reem and reemc (and could be different among
    different joint frames)
    '''
    M = generic6dReference(position.copy())
    R = M[0:3,0:3]
    R = R*comp
    M[0:3,0:3] = R
    if selec!=None:
        if isinstance(selec,str):   task.feature.selec.value = selec
        else: task.feature.selec.value = toFlags(selec)
    task.featureDes.position.value = matrixToTuple(M)
    setGain(task.gain,gain)
    if 'resetJacobianDerivative' in task.task.__class__.__dict__.keys() and resetJacobian:
        task.task.resetJacobianDerivative()

def gotoSafePos(sleeptime=5, gain=0.01):
    '''
    Go to the safe position
    In this position the hands are away from the base
    Wait some time to reach the position
    '''
    solver.clear()
    safePos = safePosition(gain)
    push(safePos)
    time.sleep(sleeptime)
    pop(safePos)

def gotoZeroPos():
    '''
    Set all the joints to 0
    Wait some time to reach the position
    '''
    solver.clear()
    zeroPos = zeroPosition()
    push(zeroPos)
    time.sleep(5)
    pop(zeroPos)
