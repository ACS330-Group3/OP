import vrep
import numpy as np


def limit(x, uLim, lLim):
    if x > uLim:
        return uLim
    if x < lLim:
        return lLim
    return x


class MotorSpeeds:
    # Class/Struct of motor speeds
    def __init__(self):
        self.fl = 0
        self.fr = 0
        self.rl = 0
        self.rr = 0

    def __repr__(self):
        return "Speeds =>\t fl: %.3f,\t fr: %.3f,\t rl: %.3f,\t rr: %.3f\n" \
               % (self.fl, self.fr, self.rl, self.rr)


class BotHandles:
    # Class/Struct of bot handles
    def __init__(self):
        self.body = 0
        self.flM = 0
        self.frM = 0
        self.rlM = 0
        self.rrM = 0
        self.botTarget = 0
        self.pathTarget = 0


class PosOrien:
    # Position Orientation Class/Struct
    def __init__(self, cl_id=-1, handle=-1):
        self.x = 0
        self.y = 0
        self.z = 0
        self.alpha = 0
        self.beta = 0
        self.gamma = 0
        self._id = cl_id
        self._handle = handle

    def __repr__(self):
        return "Pos =>\t X: %.3f,\t Y: %.3f,\t Z: %.3f\n" \
               "Rot =>\t A: %.3f,\t B: %.3f,\t G: %.3f\n" \
               % (self.x, self.y, self.z,
                  self.alpha, self.beta, self.gamma)

    def __sub__(self, other):
        _output = PosOrien()
        _output.x = self.x - other.x
        _output.y = self.y - other.y
        _output.z = self.z - other.z
        _output.alpha = ((self.alpha - other.alpha) + np.pi) % (2 * np.pi) - np.pi
        _output.beta = ((self.beta - other.beta) + np.pi) % (2 * np.pi) - np.pi
        _output.gamma = ((self.gamma - other.gamma) + np.pi) % (2 * np.pi) - np.pi
        return _output

    def __add__(self, other):
        _output = PosOrien()
        _output.x = self.x + other.x
        _output.y = self.y + other.y
        _output.z = self.z + other.z
        _output.alpha = ((self.alpha + other.alpha) + np.pi) % (2 * np.pi) - np.pi
        _output.beta = ((self.beta + other.beta) + np.pi) % (2 * np.pi) - np.pi
        _output.gamma = ((self.gamma + other.gamma) + np.pi) % (2 * np.pi) - np.pi
        return _output

    def __mul__(self, fact):
        _output = PosOrien()
        _output.x = self.x * fact
        _output.y = self.y * fact
        _output.z = self.z * fact
        _output.alpha = ((self.alpha * fact) + np.pi) % (2 * np.pi) - np.pi
        _output.beta = ((self.beta * fact) + np.pi) % (2 * np.pi) - np.pi
        _output.gamma = ((self.gamma * fact) + np.pi) % (2 * np.pi) - np.pi
        return _output

    def __div__(self, fact):
        return self.__mul__(1/fact)

    def Abs2D(self):
        positionAbs = np.sqrt(pow(self.x, 2) + pow(self.y, 2))
        rotationAbs = abs(self.gamma)
        return positionAbs, rotationAbs

    def update(self):
        if self._id != -1 and self._handle != -1:
            err_code, [self.x, self.y, self.z] = vrep.simxGetObjectPosition(self._id, self._handle, -1, vrep.simx_opmode_oneshot_wait)
            err_code, [self.alpha, self.beta, self.gamma] = vrep.simxGetObjectOrientation(self._id, self._handle, -1, vrep.simx_opmode_oneshot_wait)
        return self

    def set_pos(self, posVec):
        self.x = posVec[0]
        self.y = posVec[1]
        self.z = posVec[2]

    def set_rot(self, rotVec):
        self.alpha = rotVec[0]
        self.beta = rotVec[1]
        self.gamma = rotVec[2]

    def set(self, other):
        if self._id != -1 and self._handle != -1:
            print("Setting:\n%s" % self)
            vrep.simxSetObjectPosition(self._id, self._handle, -1, (other.x, other.y, other.z), vrep.simx_opmode_oneshot)
            vrep.simxSetObjectOrientation(self._id, self._handle, -1, (other.alpha, other.beta, other.gamma), vrep.simx_opmode_oneshot)
            self.x = other.x
            self.y = other.y
            self.z = other.z
            self.alpha = other.alpha
            self.beta = other.beta
            self.gamma = other.gamma
        return self


class PidGains:
    def __init__(self):
        self.p = 0
        self.i = 0
        self.d = 0


class Pid:
    def __init__(self):
        self._k = PidGains()
        self._prevError = 0
        self._iError = 0
        self.iLim = 50

    def set_gains(self, newK):
        self._k = newK

    def reset(self):
        self._prevError = 0
        self._iError = 0

    def step(self, error):
        dError = error - self._prevError
        self._iError = self._iError + error
        if abs(self._iError) > self.iLim:
            self._iError = self.iLim * (self._iError / abs(self._iError))
        return self._k.p * error + self._k.i * self._iError + self._k.d * dError
