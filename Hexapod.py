import vrep
from math import *
import time
import sys


class Joint:
    def __init__(self, client_id, name):
        self.client_id = client_id
        self.name = name
        err, self.handle = vrep.simxGetObjectHandle(client_id, name, vrep.simx_opmode_oneshot_wait)
        vrep.simxGetJointPosition(self.client_id, self.handle, vrep.simx_opmode_streaming)

    def get_angle(self):
        err, angle = vrep.simxGetJointPosition(self.client_id, self.handle, vrep.simx_opmode_buffer)
        return angle

    def set_angle(self, angle):
        vrep.simxSetJointTargetPosition(self.client_id, self.handle, -angle, vrep.simx_opmode_oneshot)


class Leg:
    l1 = 0.05
    l2 = 0.0725
    l3 = 0.1113

    # Initial position
    x0 = (l1 + l2 + l3) / 2
    y0 = 0
    z0 = -(l2 + l3) / 2

    def __init__(self, j1, j2, j3):
        self.j1 = j1
        self.j2 = j2
        self.j3 = j3
        self.orientation = 0

        self.q1 = self.j1.get_angle()
        self.q2 = self.j2.get_angle()
        self.q3 = self.j3.get_angle()

        self.x = self.x0
        self.y = self.y0
        self.z = self.z0

        # shifting coordinates
        self.dx = 0
        self.dy = 0

        self.update()

    def update(self):
        self.ozk()
        # self.pzk()

        self.j1.set_angle(self.q1)
        self.j2.set_angle(self.q2)
        self.j3.set_angle(self.q3)

    # inverse kinematics
    def ozk(self):
        a = sqrt(self.x**2 + self.y**2) - self.l1  # XY - l1
        b = sqrt(a**2 + self.z**2)  # C

        self.q1 = atan2(self.y, self.x)
        self.q2 = acos(a / b) - acos((self.l2**2 + b**2 - self.l3**2) / (2 * b * self.l2))
        self.q3 = pi - acos((self.l3**2 + self.l2**2 - b**2) / (2 * self.l2 * self.l3))

        if self.z < 0:
            self.q2 = -self.q2
            self.q3 = -self.q3

    # align coordinate system of a leg with other legs according to its orientation
    def align(self, angle, step=0.1):
        h = step / 2
        angle += radians(self.orientation)

        self.dx = - h * sin(angle)
        self.dy = h * cos(angle)

    def shift(self, t):  # t <= 1.991 for step = 0.1; FAST: t = 0.28 for step 0.7 in align(...)
        self.x = self.x0 + self.dx * t
        self.y = self.y0 + self.dy * t

        self.update()

    def __str__(self):
        return "j1 = {}, j2 = {}, j3 = {}, orientation = {}".format(self.j1.name,
                                                                    self.j2.name,
                                                                    self.j3.name,
                                                                    self.orientation)


class Hexapod:
    angles = [120, 180, -120, -60, 0, 60]
    X = 0
    Y = 0
    Z = 0

    def __init__(self, legs):
        self.legs = legs
        for i in range(0, len(self.angles)):
            legs[i].orientation = self.angles[i]
        self.change_orientation(radians(0))

    def change_orientation(self, angle, step=0.1):
        for leg in self.legs:
            leg.align(angle, step)

    def shift_all(self, t):
        for leg in self.legs:
            leg.shift(t)

    def body_shift(self, step=30):
        angle = 0
        while angle < radians(360):
            i = 0
            self.change_orientation(angle)
            while i < 1.8:  # i means depth of shift
                self.shift_all(i)
                # time.sleep(0.001)
                i += 0.001
            while i > 0:
                self.shift_all(i)
                # time.sleep(0.001)
                i -= 0.001
            angle += radians(step)

    def step_forward(self, a, b, alpha=0):  # "is_even" checks which legs are going up
        self.change_orientation(radians(alpha))
        i = 0
        while i < 1:
            i += 0.001
            for j in range(a, len(self.legs), 2):
                reducer = (1 - i) if i < 0.5 else i  # half way up and half way down
                self.legs[j].z = self.legs[j].z0 * reducer  # z0 = -0.0919
                self.legs[j].shift(-i)
            for j in range(b, len(self.legs), 2):
                self.legs[j].shift(i)
            # time.sleep(0.001)
        while i > 0:
            i -= 0.001
            for j in range(b, len(self.legs), 2):
                reducer = (1 - i) if i < 0.5 else i  # half way up and half way down
                self.legs[j].z = self.legs[j].z0 * reducer  # z0 = -0.0919
                self.legs[j].shift(i)
            for j in range(a, len(self.legs), 2):
                self.legs[j].shift(-i)
            # time.sleep(0.001)

    def walk(self, gait="tripod", step_length=1.0, step_height=0.5, inhibitor=0.01, work_time=10):
        t = [0.0, 0.0]  # multipliers for forward and backward leg moves
        max_step_length = 2.0
        pace = max_step_length * step_length
        turning_point = pace / 4
        step = 0.01
        z_counter = 0.0  # counter for z axis control
        h = abs(Leg.z0 * step_height)  # step height
        lean = True  # firstly, hexapod should lean from initial position to start walking
        even = True  # chooses which legs are going up and forward: even (0, 2, 4) or odd (1, 3, 5)
        first_step = True

        if gait == "tripod":
            print("tripod gait on")
            start = time.time()
            while time.time() - start < work_time:
                if lean:  # lean forward
                    t[0] += step
                    t[1] += step
                    if t[0] >= turning_point or t[1] >= turning_point:
                        lean = False
                else:  # increase/decrease multipliers to move even and odd legs in different directions
                    if t[0] <= -turning_point or t[1] <= -turning_point:
                        even = not even
                        z_counter = 0.0
                    z_counter += step
                    t[int(not even)] -= step
                    t[int(even)] += step

                # shifting legs
                for i in range(0, len(self.legs)):
                    if i % 2 == int(not even):
                        # move legs off the ground
                        if not lean:
                            x = z_counter * 2 if first_step else z_counter
                            # self.legs[i].z = self.legs[i].z0 + (-(sqrt(h) / m * y - sqrt(h)) ** 2 + h)
                            self.legs[i].z = self.legs[i].z0 + h * x / step_length * (2 - x / step_length)
                        self.legs[i].shift(t[int(not even)])
                    else:
                        # move legs on the ground
                        self.legs[i].shift(t[int(even)])

                # switch off first step mode
                if first_step and round(z_counter, 2) >= pace / 2:
                    first_step = False
                time.sleep(inhibitor)

        elif gait == "wave":
            print("wave gait on")

        elif gait == "ripple":
            print("ripple gait on")
        else:
            sys.exit("Wrong gait name")

        # return to initial position
        for i in range(0, len(self.legs)):
            self.legs[i].shift(0)
