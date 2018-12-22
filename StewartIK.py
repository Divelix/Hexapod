import vrep
import vrepConst
import sys
import time
import numpy as np
from sympy import symbols, Matrix

vrep.simxFinish(-1)
client_id = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

if client_id != -1:
    print('Connected to remote API server')
    vrep.simxStartSimulation(client_id, vrep.simx_opmode_blocking)
else:
    sys.exit('Failed connecting to remote API server')


class Joint:
    def __init__(self, client_id, name):
        self.client_id = client_id
        self.name = name
        err, self.handle = vrep.simxGetObjectHandle(client_id, name, vrep.simx_opmode_oneshot_wait)
        vrep.simxGetJointPosition(self.client_id, self.handle, vrep.simx_opmode_streaming)

    def get_length(self):
        err, length = vrep.simxGetJointPosition(self.client_id, self.handle, vrep.simx_opmode_buffer)
        return length

    def set_length(self, length):
        vrep.simxSetJointTargetPosition(self.client_id, self.handle, -length, vrep.simx_opmode_oneshot)


class Platform:
    def __init__(self):
        p1 = Joint(client_id, 'L1')
        p2 = Joint(client_id, 'L2')
        p3 = Joint(client_id, 'L3')
        p4 = Joint(client_id, 'L4')
        p5 = Joint(client_id, 'L5')
        p6 = Joint(client_id, 'L6')
        self.legs = np.array([p1, p2, p3, p4, p5, p6])
        self.b = np.array([[0, 0.333,  0.133,  -0.533,  -0.733,  -0.4],
                           [0, 0.5768, 0.9232,  0.9232, 0.5768,   0],
                           [0, 0,      0,       0,       0,       0]])

        self.p = np.array([[0.2098,   0.3098, 0.1, -0.1, -0.3098, -0.2098],
                           [-0.2366, -0.0634, 0.3,  0.3, -0.0634, -0.2366],
                           [0,        0,      0,    0,    0,       0]])

    def move_platform(self, P, R):
        for i in range(6):
            leg_vec = P + np.dot(R, self.p[:, i]) - self.b[:, i]
            len = np.linalg.norm(leg_vec)
            self.legs[i].set_length(len - 0.55)

    def move_all(self, angle):
        for leg in self.legs:
            leg.set_length(angle)

    def print_all(self):
        for leg in self.legs:
            print(leg.get_length())


R1 = np.array([[1, 0, 0],
               [0, 1, 0],
               [0, 0, 1]])
P1 = np.array([-0.2, 0.5, 0.6])
stew = Platform()
print("Start")
while True:
    stew.move_platform(P1, R1)
    time.sleep(1)
    i = 0
    while i < 16*np.pi:
        x = 0.05*np.sin(i)
        y = 0.05*np.cos(i)
        P = np.array([-0.2 + x, 0.5 + y, 0.6])
        stew.move_platform(P, R1)
        i += 0.1
        time.sleep(0.01)
    stew.move_platform(P1, R1)
    time.sleep(1)
    i = 0
    while i < 16*np.pi:
        x = 0.05*np.sin(i)
        y = 0.05*np.cos(i)
        z = 0.1*np.sin(0.5*i)
        P = np.array([-0.2 + x, 0.5 + y, 0.6 + z])
        stew.move_platform(P, R1)
        i += 0.1
        time.sleep(0.01)
    stew.move_platform(P1, R1)
    time.sleep(1)
    i = 0
    while i < 16*np.pi:
        amp = 0.25
        x = amp*np.sin(i)
        y = amp*np.cos(i)
        R = np.array([[1, x, x],
                      [x, 1, y],
                      [y, y, 1]])
        P = np.array([-0.2, 0.5, 0.6])
        stew.move_platform(P, R)
        i += 0.1
        time.sleep(0.01)
    stew.move_platform(P1, R1)
print("End")
