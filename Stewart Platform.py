from Hexapod import *
import vrep
import vrepConst
import sys
import time
import numpy as np
from sympy import symbols, Matrix

vrep.simxFinish(-1)
client_id = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

if client_id != -1:
    print 'Connected to remote API server'
    vrep.simxStartSimulation(client_id, vrep.simx_opmode_blocking)
else:
    sys.exit('Failed connecting to remote API server')

err, pos_h = vrep.simxGetObjectHandle(client_id, 'Disc', vrep.simx_opmode_oneshot_wait)

class Platform:
    def __init__(self):
        self.p1 = Joint(client_id, 'Prismatic_Joint_1')
        self.p2 = Joint(client_id, 'Prismatic_Joint_2')
        self.p3 = Joint(client_id, 'Prismatic_Joint_3')
        self.p4 = Joint(client_id, 'Prismatic_Joint_4')
        self.p5 = Joint(client_id, 'Prismatic_Joint_5')
        self.p6 = Joint(client_id, 'Prismatic_Joint_6')

    def move(self, angle):
        self.p1.set_angle(angle)
        self.p2.set_angle(angle)
        self.p3.set_angle(angle)
        self.p4.set_angle(angle)
        self.p5.set_angle(angle)
        self.p6.set_angle(angle)
        self.l_real = self.l_base + angle

    p = np.array([[0, 0.29283, 0.69284, 0.54642, -0.25359, -0.40001],
                  [0, 0,       0.69283, 0.94643,  0.94643,  0.69283],
                  [0, 0,       0,       0,        0,        0]])

    b = np.array([[0, 0.63015, 0.98625, 0.67117, -0.04101, -0.35609],
                  [0, 0,       0.61677, 1.16250,  1.16250,  0.61677],
                  [0, 0,       0,       0,        0,        0]])

    nx, ny, nz, ox, oy, oz, tx, ty, tz = symbols('nx ny nz ox oy oz tx ty tz', real=True)

    l = 0.1
    l_base = np.array([0.25, 0.25, 0.25, 0.25, 0.25, 0.25])
    l_real = l_base + l

    vector = np.array([0.33, 0.33, 0.33, 1, 1, 1, 0, 0, 10])

    A1 = b[0][1] / p[0][1]
    A2 = b[0][1]

    B1 = (b[0][2] * p[0][1] - b[0][1] * p[0][2]) / (p[0][1] * p[1][2])
    B2 = ((b[0][2] - b[0][1]) * p[0][2]) / p[1][2]
    B3 = b[0][2]
    B4 = b[1][2] / p[1][2]
    B5 = (b[1][2] * p[0][2]) / p[1][2]
    B6 = b[1][2]

    def K(self, i):
        return (self.l_real[i]**2 - self.b[0][i] ** 2 - self.b[1][i] ** 2 - self.p[0][i] ** 2 - self.p[1][i] ** 2 - self.l_real[0] ** 2) / 2

    def C1(self, i):
        return self.A1 * self.p[0][i] + self.B1 * self.p[1][i] - self.b[0][i]
    def C2(self, i):
        return self.A2 * self.p[0][i] + self.B2 * self.p[1][i] - self.b[0][i] * self.p[0][i]
    def C3(self, i):
        return self.B3 * self.p[1][i] - self.b[0][i] * self.p[1][i]
    def C4(self, i):
        return self.B4 * self.p[1][i] - self.b[1][i]
    def C5(self, i):
        return self.B5 * self.p[1][i] - self.b[1][i] * self.p[0][i]
    def C6(self, i):
        return self.B6 * self.p[1][i] - self.b[1][i] * self.p[1][i]
    def C(self, i):
        return self.K(i) - (self.K(1) / self.p[0][1]) * self.p[0][i] - ((self.K(2) * self.p[0][1] - self.K(1) * self.p[0][2]) / (self.p[0][1] * self.p[1][2])) * self.p[1][i]

    def constant(self):
        A = self.K(1) / self.p[0][1]
        B = (self.K(2) * self.p[0][1] - self.K(1) * self.p[0][2]) / (self.p[0][1] * self.p[1][2])
        F1 = self.nx ** 2 + self.ny ** 2 + self.nz ** 2 - 1
        F2 = self.ox ** 2 + self.oy ** 2 + self.oz ** 2 - 1
        F3 = self.tx ** 2 + self.ty ** 2 + self.tz ** 2 - self.l_real[0] ** 2
        F4 = self.nx * self.ox + self.ny * self.oy + self.nz * self.oz
        F5 = self.nx * self.tx + self.ny * self.ty + self.nz * self.tz - self.A1 * self.tx - self.A2 * self.nx - A
        F6 = self.ox * self.tx + self.oy * self.ty + self.oz * self.tz - self.B1 * self.tx - self.B2 * self.nx - self.B3 * self.ox - self.B4 * self.ty - self.B5 * self.ny - self.B6 * self.oy - B
        F7 = self.C1(3) * self.tx + self.C2(3) * self.nx + self.C3(3) * self.ox + self.C4(3) * self.ty + self.C5(3) * self.ny + self.C6(3) * self.oy - self.C(3)
        F8 = self.C1(4) * self.tx + self.C2(4) * self.nx + self.C3(4) * self.ox + self.C4(4) * self.ty + self.C5(4) * self.ny + self.C6(4) * self.oy - self.C(4)
        F9 = self.C1(5) * self.tx + self.C2(5) * self.nx + self.C3(5) * self.ox + self.C4(5) * self.ty + self.C5(5) * self.ny + self.C6(5) * self.oy - self.C(5)

        self.F = F1**2 + F2**2 + F3**2 + F4**2 + F5**2 + F6**2 +F7**2 + F8**2 + F9**2
        self.R = Matrix([[F1], [F2], [F3], [F4], [F5], [F6], [F7], [F8], [F9]])
        self.J = self.R.jacobian([self.nx, self.ny, self.nz, self.ox, self.oy, self.oz, self.tx, self.ty, self.tz])

    def F0(self, X):
        return np.array(self.F.subs(
            [(self.nx, X[0]), (self.ny, X[1]), (self.nz, X[2]),
             (self.ox, X[3]), (self.oy, X[4]), (self.oz, X[5]),
             (self.tx, X[6]), (self.ty, X[7]), (self.tz, X[8])])).astype(np.float64)

    def R0(self, X):
        return np.array(self.R.subs(
            [(self.nx, X[0]), (self.ny, X[1]), (self.nz, X[2]),
             (self.ox, X[3]), (self.oy, X[4]), (self.oz, X[5]),
             (self.tx, X[6]), (self.ty, X[7]), (self.tz, X[8])])).astype(np.float64)

    def J0(self, X):
        return np.array(self.J.subs(
            [(self.nx, X[0]), (self.ny, X[1]), (self.nz, X[2]),
             (self.ox, X[3]), (self.oy, X[4]), (self.oz, X[5]),
             (self.tx, X[6]), (self.ty, X[7]), (self.tz, X[8])])).astype(np.float64)

    def get_position(self):
        t0 = time.time()
        self.constant()
        J1 = self.J0(self.vector)
        v = 2
        m = 1e-3 * np.max(np.dot(J1.T, J1))

        i = 0
        while i < 5 and self.F0(self.vector) > 1e-6:
            R1 = self.R0(self.vector)
            J1 = self.J0(self.vector)

            A = np.dot(J1.T, J1)

            g = np.dot(J1.T, R1)

            E = np.eye(A.shape[0], A.shape[1])

            leftPartInverse = np.linalg.inv(A + m * E)

            d_lm = -np.dot(leftPartInverse, g)

            vector_new = self.vector.reshape(-1, 1) + d_lm
            R2 = self.R0(vector_new)

            grain_numerator = (0.5 * np.dot(R1.T, R1)) - (0.5 * np.dot(R2.T, R2))

            gain_divisor = 0.5 * (np.dot(d_lm.T, (m * d_lm - g))) + 1e-10

            gain = grain_numerator / gain_divisor

            if gain > 0:
                self.vector = vector_new
                m = m * np.max([1 / 3, 1 - (2 * gain - 1) ** 3])
            else:
                m = m * v
                v = v * 2

            i += 1

        t1 = time.time()
        print 'i:', i, '    time:', t1 - t0, '    F:', self.F0(self.vector)
        # print self.vector[8] + 1.0389e-1


stew = Platform()

i = 0
while True:
        i += 0.05
        stew.move(0.2 * sin(i) + 0.2)
        pos = vrep.simxGetObjectPosition(client_id, pos_h, -1, vrep.simx_opmode_oneshot)

        stew.get_position()
        print 'error:', stew.vector[8] - pos[1][2] + 1.0389e-1
