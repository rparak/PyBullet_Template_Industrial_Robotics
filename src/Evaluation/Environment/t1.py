import pybullet as p
import time
import math
import pybullet_data



#cid = p.connect(p.SHARED_MEMORY_GUI)
cid = p.connect(p.GUI)
if (cid < 0):
  p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setPhysicsEngineParameter(numSolverIterations=10)
p.setTimeStep(1. / 100.)
logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "visualShapeBench.json")
planeId = p.loadURDF("plane_transparent.urdf", globalScaling=0.5, useMaximalCoordinates=True, useFixedBase=True)
# checker_blue.png, checker_grid.jpg
texUid = p.loadTexture("checker_blue.png")
p.changeVisualShape(planeId, -1, textureUniqueId=texUid)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
p.configureDebugVisualizer(p.COV_ENABLE_PLANAR_REFLECTION, 1)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)

p.loadURDF("cube.urdf", [  0,   0, 1], globalScaling=0.1)
p.loadURDF("cube.urdf", [  0, 0.5, 1], globalScaling=0.1, useFixedBase=True)
p.loadURDF("cube.urdf", [0.5,   0, 1], globalScaling=0.1)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
#p.stopStateLogging(logId)
p.setGravity(0, 0, -10)
#p.setRealTimeSimulation(1)

while (1):
  #p.getDebugVisualizerCamera()
  p.stepSimulation()