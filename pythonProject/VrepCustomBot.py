import math
import vrep
import numpy as np
import sys


class MotorSpeeds:
    # Class/Struct of motor speeds
    def __init__(self):
        self.fl = 0
        self.fr = 0
        self.rl = 0
        self.rr = 0

    def __repr__(self):
        return "Speeds =>\t fl: %d,\t fr: %d,\t rl: %d,\t rr: %d\n" \
               % (self.fl, self.fr, self.rl, self.rr)


class BotHandles:
    # Class/Struct of bot handles
    def __init__(self):
        self.body = 0
        self.flM = 0
        self.frM = 0
        self.rlM = 0
        self.rrM = 0


class PosOrien:
    # Position Orientation Class/Struct
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.alpha = 0
        self.beta = 0
        self.gamma = 0

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

    def update(self, clientId, handle):
        err_code, [self.x, self.y, self.z] = vrep.simxGetObjectPosition(clientId, handle, -1, vrep.simx_opmode_oneshot_wait)
        err_code, [self.alpha, self.beta, self.gamma] = vrep.simxGetObjectOrientation(clientId, handle, -1, vrep.simx_opmode_oneshot_wait)
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

class VrepBot:
    def __init__(self):
        self.clientId = -1
        self.handles = BotHandles()
        self.speeds = MotorSpeeds()
        self.po = PosOrien()
        self.targetPO = PosOrien()
        self.xPid = Pid()
        self.yPid = Pid()
        self.gammaPid = Pid()

    def setup(self):
        self.connect()

    def connect(self, _port=19999):
        vrep.simxFinish(-1)
        self.clientId = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

        if self.clientId != -1:
            print("Connection on port: {}, successful - ID: {}".format(_port, self.clientId))
        else:
            print("Connection on port: {}, failed".format(_port))
            inp = raw_input("Press 'q' to exit, press enter to continue\n")
            if inp == 'q':
                print("Exiting")
                sys.exit()
            print("Continuing")

        self.get_handles()
        self.get_pos_orien()
        return self.clientId != -1

    def get_handles(self):
        err_code, self.handles.body = vrep.simxGetObjectHandle(self.clientId, "ME_Platfo2_sub1", vrep.simx_opmode_oneshot_wait)
        err_code, self.handles.flM = vrep.simxGetObjectHandle(self.clientId, "rollingJoint_fl", vrep.simx_opmode_oneshot_wait)
        err_code, self.handles.frM = vrep.simxGetObjectHandle(self.clientId, "rollingJoint_fr", vrep.simx_opmode_oneshot_wait)
        err_code, self.handles.rlM = vrep.simxGetObjectHandle(self.clientId, "rollingJoint_rl", vrep.simx_opmode_oneshot_wait)
        err_code, self.handles.rrM = vrep.simxGetObjectHandle(self.clientId, "rollingJoint_rr", vrep.simx_opmode_oneshot_wait)
        return self.handles

    def calc_motors(self, xVel, yVel, angVel):
        ROT_SCALE_FACT = 0.3448  # experimentally scale to 1 = 1 rad/s
        VEL_SCALE_FACT = -10  # expecimentally scale to 1 = 1 m/s
        LIN_VEL_MAX = 1
        ANG_VEL_MAX = LIN_VEL_MAX/ROT_SCALE_FACT

        magAngVel = abs(angVel)
        if magAngVel > ANG_VEL_MAX:
            angVel = angVel / magAngVel

        magLinVel = math.sqrt(pow(xVel, 2) + pow(yVel, 2))
        if magLinVel > LIN_VEL_MAX:
            xVel = LIN_VEL_MAX * xVel / magLinVel
            yVel = LIN_VEL_MAX * yVel / magLinVel
            # print("Lim, factor: %.3f" % magLinVel)

        # print("x: %.3f\t y: %.3f\t g: %.3f" % (xVel, yVel, angVel))

        fbVel = +xVel * math.cos(self.po.gamma + math.pi/2) + yVel * math.sin(self.po.gamma + math.pi/2)
        lrVel = -xVel * math.sin(self.po.gamma + math.pi/2) + yVel * math.cos(self.po.gamma + math.pi/2)

        # print("x: %.3f\t y: %.3f\t g: %.3f\n" % (fbVel, lrVel, angVel))

        self.speeds.fl = VEL_SCALE_FACT * (fbVel - lrVel - (ROT_SCALE_FACT) * angVel)
        self.speeds.fr = VEL_SCALE_FACT * (fbVel + lrVel + (ROT_SCALE_FACT) * angVel)
        self.speeds.rl = VEL_SCALE_FACT * (fbVel + lrVel - (ROT_SCALE_FACT) * angVel)
        self.speeds.rr = VEL_SCALE_FACT * (fbVel - lrVel + (ROT_SCALE_FACT) * angVel)
        return self.speeds

    def set_motors(self, blocking=False):
        """
        bot control is improved with non-blocking commands (simultaneous motor adjustment)
        this breaks at end of script - terminates before command received so add optional blocking param for stop()
         - waits for duplicate command to be sent
        """
        vrep.simxSetJointTargetVelocity(self.clientId, self.handles.flM, self.speeds.fl, vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetVelocity(self.clientId, self.handles.frM, self.speeds.fr, vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetVelocity(self.clientId, self.handles.rlM, self.speeds.rl, vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetVelocity(self.clientId, self.handles.rrM, self.speeds.rr, vrep.simx_opmode_oneshot)
        if blocking:
            # send duplicate and wait for complete
            vrep.simxSetJointTargetVelocity(self.clientId, self.handles.rrM, self.speeds.rr, vrep.simx_opmode_oneshot_wait)

    def get_pos_orien(self):
        return self.po.update(self.clientId, self.handles.body)

    def stop(self, force=True):
        self.speeds = MotorSpeeds()
        self.set_motors(force)

    def set_target(self, posOrien):
        self.targetPO = posOrien
        # reset PID?

    def target_step(self, updatePO=False):
        if updatePO:
            error = self.targetPO - self.get_pos_orien()
        else:
            error = self.targetPO - self.po
        # print("Error:\n%s\n" % error)  # add for Error printout
        xVel = self.xPid.step(error.x)
        yVel = self.yPid.step(error.y)
        gammaVel = self.gammaPid.step(error.gamma)
        self.calc_motors(xVel, yVel, gammaVel)
        self.set_motors()