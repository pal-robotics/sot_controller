from sot_ros_api import *
from dynamic_graph.sot.pattern_generator import *
from dynamic_graph.ros.ros_sot_pattern_generator import RosSotPatternGenerator
from dynamic_graph.ros.ros_sot_robot_model import RosSotRobotModel
pg = RosSotPatternGenerator('pg')
pg.loadFromParameterServer()
pg.buildModelFromUrdf()

pg.parseCmd(":samplingperiod 0.005")
pg.parseCmd(":previewcontroltime 1.6")
pg.parseCmd(":comheight 0.814")
pg.parseCmd(":omega 0.0")
pg.parseCmd(":stepheight 0.05")
pg.parseCmd(":singlesupporttime 0.780")
pg.parseCmd(":doublesupporttime 0.020")
pg.parseCmd(":armparameters 0.5")
pg.parseCmd(":LimitsFeasibility 0.0")
pg.parseCmd(":ZMPShiftParameters 0.015 0.015 0.015 0.015")
pg.parseCmd(":TimeDistributeParameters 2.0 3.5 1.0 3.0")
pg.parseCmd(":UpperBodyMotionParameters 0.0 -0.5 0.0")
pg.parseCmd(":comheight 0.814")
pg.parseCmd(":SetAlgoForZmpTrajectory Morisawa")

plug(robot.dynamic.position,pg.position)
plug(robot.dynamic.com,pg.com)
pg.motorcontrol.value = robot.dynamic.getDimension()*(0,)
pg.zmppreviouscontroller.value = (0,0,0)

pg.initState()

geom = RosSotRobotModel('geom')
geom.setNamespace('sot_controller')
geom.loadFromParameterServer()

geom.createOpPoint('rf','right-ankle')
geom.createOpPoint('lf','left-ankle')
plug(robot.dynamic.position,geom.position)
geom.ffposition.value = 6*(0,)
geom.velocity.value = robot.dynamic.getDimension()*(0,)
geom.acceleration.value = robot.dynamic.getDimension()*(0,)


comRef = Selector('comRef',['vector','ref',robot.dynamic.com,pg.comref])
plug(pg.inprocess,comRef.selec)

selecSupportFoot = Selector('selecSupportFoot',['matrixHomo','pg_H_sf',pg.rightfootref,pg.leftfootref],['matrixHomo','wa_H_sf',geom.rf,geom.lf])
plug(pg.SupportFoot,selecSupportFoot.selec)
sf_H_wa = Inverse_of_matrixHomo('sf_H_wa')
plug(selecSupportFoot.wa_H_sf,sf_H_wa.sin)
pg_H_wa = Multiply_of_matrixHomo('pg_H_wa')
plug(selecSupportFoot.pg_H_sf,pg_H_wa.sin1)
plug(sf_H_wa.sout,pg_H_wa.sin2)

wa_H_pg = Inverse_of_matrixHomo('wa_H_pg')
plug(pg_H_wa.sout,wa_H_pg.sin)
wa_zmp = Multiply_matrixHomo_vector('wa_zmp')
plug(wa_H_pg.sout,wa_zmp.sin1)
plug(pg.zmpref,wa_zmp.sin2)

pg.parseCmd(':SetZMPFrame world')
plug(wa_zmp.sout,robot.device.zmp)



taskWaist = MetaTask6d('waist',robot.dynamic,'base_joint','base_joint')

waistReferenceVector = Stack_of_vector('waistReferenceVector')
plug(pg.initwaistposref,waistReferenceVector.sin1)
plug(pg.initwaistattref,waistReferenceVector.sin2)
waistReferenceVector.selec1(0,3)
waistReferenceVector.selec2(0,3)
waistReference=PoseRollPitchYawToMatrixHomo('waistReference')
plug(waistReferenceVector.sout,waistReference.sin)
plug(waistReference.sout,taskWaist.featureDes.position)

taskWaist.feature.selec.value = '011100'
taskWaist.task.controlGain.value = 5

featureCom = FeatureGeneric('featureCom')
plug(robot.dynamic.com,featureCom.errorIN)
plug(robot.dynamic.com.Jcom,featureCom.jacobianIN)
featureComDes = FeatureGeneric('featureComDes')
plug(comRef.ref,featureComDes.errorIN)
featureCom.selec.value = '011'

taskComPD = TaskPD('taskComPD')
taskComPD.add('featureCom')
plug(pg.dcomref,featureComDes.errordotIN)
plug(featureCom.errordot,taskComPD.errorDot)
taskComPD.controlGain.value = 40
taskComPD.setBeta(-1)

taskRF = MetaTask6d('rf',robot.dynamic,'right_sole_joint','right_sole_joint')
taskLF = MetaTask6d('lf',robot.dynamic,'left_sole_joint','left_sole_joint')

plug(pg.rightfootref,taskRF.featureDes.position)
taskRF.task.controlGain.value = 5
plug(pg.leftfootref,taskLF.featureDes.position)
taskLF.task.controlGain.value = 5

push(taskWaist)
push(taskRF)
push(taskLF)
push(taskComPD)

plug(solver.control,robot.device.control)


pg.parseCmd(':SetAlgoForZmpTrajectory Herdt')
pg.parseCmd(':doublesupporttime 0.1')
pg.parseCmd(':singlesupporttime 0.7')

pg.parseCmd(':numberstepsbeforestop 4')

pg.parseCmd(':setfeetconstraint XY 0.09 0.06')

pg.parseCmd(':HerdtOnline 0.1 0.0 0.0')

pg.velocitydes.value =(0.1,0.0,0.0)






