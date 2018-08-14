from Hexapod import *
import vrep
import vrepConst
import sys
import time

vrep.simxFinish(-1)
client_id = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

if client_id != -1:
    print 'Connected to remote API server'
    vrep.simxStartSimulation(client_id, vrep.simx_opmode_blocking)
else:
    sys.exit('Failed connecting to remote API server')

# convention of naming joints:
# L - left, R - right
# F - front, M - middle, B - back
# 1 - coxa, 2 - femur, 3 - tibia
group = vrep.simxGetObjectGroupData(client_id, vrepConst.sim_object_joint_type, 0, vrep.simx_opmode_blocking)
names = group[4]
print "Names: ", names

joints = []
legs = []
for i in xrange(0, 18):
    joints.append(Joint(client_id, names[i]))

for i in xrange(0, len(joints), 3):
    legs.append(Leg(joints[i], joints[i+1], joints[i+2]))

hexapod = Hexapod(legs)

time.sleep(1)
# hexapod.body_shift(30)
# hexapod.step_forward(0, 1, 45)
# hexapod.step_forward(1, 0, -45)
# hexapod.change_orientation(120)
hexapod.walk(gait="tripod", step_length=1.0, step_height=0.8, inhibitor=0.005, work_time=6)
hexapod.walk(gait="tripod", step_length=0.1, step_height=0.8, inhibitor=0.01, work_time=6)
time.sleep(2)

for leg in legs:
    print "{}: x = {}; y = {}; z = {}".format(leg.j3.name, leg.x, leg.y, leg.z)

vrep.simxStopSimulation(client_id, vrep.simx_opmode_blocking)
