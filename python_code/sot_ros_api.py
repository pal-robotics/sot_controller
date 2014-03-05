# Create the reem robot model and a solver for the stack of tasks

print('loading sot robot model')
from sot_robot.prologue import robot, solver

print('robot got created')

# Useful modules to interact with the stack of tasks
# They should be shared between the demos...
from dynamic_graph.sot.dyninv import *
from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple,rotate, matrixToRPY
from dynamic_graph import writeGraph

# User defined modules
from utilities.kinematics import *
from utilities.sot import *
from utilities.tasks import *
from utilities.rosinterface import *
from dynamic_graph.dynamic_graph_signal_convert import DynamicGraphSignalConverter
# Python modules
import time
import numpy

# Ros modules
import roslib; roslib.load_manifest('sot_controller')
import tf
import rospy

"""
To show the dynamic graph:
np.set_printoptions(suppress=True, precision=3)
python: writeGraph ("/tmp/graph.dot")

shell: dot -o graph.pdf -Tpdf graph.dot
"""
def createPdf(name='graph', folder='/tmp/'):
    file = folder + name
    writeGraph(file+'.dot')
    import os
    os.system('dot -o '+file+'.pdf -Tpdf '+file+'.dot')
    os.system('evince '+file+'.pdf &')
