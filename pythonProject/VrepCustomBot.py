import sys
import copy
import time
from VrepBotTools import *


class VrepBot:
    def __init__(self):
        self.clientId = -1
        self.connect()

        self.handles = BotHandles()
        self.get_handles()

        self.po = PosOrien(self.clientId, self.handles.body)
        self.targetPO = PosOrien(self.clientId, self.handles.botTarget)
        self.pathTargetPO = PosOrien(self.clientId, self.handles.pathTarget)
        self.po.update()
        self.targetPO.update()
        self.pathTargetPO.update()

        self.speeds = MotorSpeeds()
        self.xPid = Pid()
        self.yPid = Pid()
        self.gammaPid = Pid()

        self.tPrev = 0
        self.tDuty = 0.1

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

        return self.clientId != -1

    def check_connected(self):
        return self.clientId != -1

    def get_handles(self):
        err_code, self.handles.body = vrep.simxGetObjectHandle(self.clientId, "ME_Platfo2_sub1", vrep.simx_opmode_oneshot_wait)
        err_code, self.handles.flM = vrep.simxGetObjectHandle(self.clientId, "rollingJoint_fl", vrep.simx_opmode_oneshot_wait)
        err_code, self.handles.frM = vrep.simxGetObjectHandle(self.clientId, "rollingJoint_fr", vrep.simx_opmode_oneshot_wait)
        err_code, self.handles.rlM = vrep.simxGetObjectHandle(self.clientId, "rollingJoint_rl", vrep.simx_opmode_oneshot_wait)
        err_code, self.handles.rrM = vrep.simxGetObjectHandle(self.clientId, "rollingJoint_rr", vrep.simx_opmode_oneshot_wait)
        err_code, self.handles.botTarget = vrep.simxGetObjectHandle(self.clientId, "OmniTarget", vrep.simx_opmode_oneshot_wait)
        err_code, self.handles.pathTarget = vrep.simxGetObjectHandle(self.clientId, "PathTarget", vrep.simx_opmode_oneshot_wait)
        return self.handles

    def calc_motors(self, xVel, yVel, angVel):
        ROT_SCALE_FACT = 0.3448  # experimentally scale to 1 = 1 rad/s
        VEL_SCALE_FACT = -10  # expecimentally scale to 1 = 1 m/s
        WHEEL_MAX_ROT = 4 * np.pi / 3

        # Convert to m/s & rad/s
        xVel = xVel * VEL_SCALE_FACT
        yVel = yVel * VEL_SCALE_FACT
        angVel = angVel * VEL_SCALE_FACT * ROT_SCALE_FACT

        fbVel = +xVel * np.cos(self.po.gamma) + yVel * np.sin(self.po.gamma)
        lrVel = -xVel * np.sin(self.po.gamma) + yVel * np.cos(self.po.gamma)

        fbRot = limit(fbVel, WHEEL_MAX_ROT, -WHEEL_MAX_ROT)
        lrRot = limit(lrVel, WHEEL_MAX_ROT, -WHEEL_MAX_ROT)
        angRot = limit(angVel, WHEEL_MAX_ROT, -WHEEL_MAX_ROT)

        self.speeds.fl = fbRot - lrRot - angRot
        self.speeds.fr = fbRot + lrRot + angRot
        self.speeds.rl = fbRot + lrRot - angRot
        self.speeds.rr = fbRot - lrRot + angRot
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

    def stop(self, force=True):
        self.speeds = MotorSpeeds()
        self.set_motors(force)

    def target_step(self, updatePO=False):
        if updatePO:
            error = self.targetPO.update() - self.po.update()
        else:
            error = self.targetPO - self.po
        xVel = self.xPid.step(error.x)
        yVel = self.yPid.step(error.y)
        gammaVel = self.gammaPid.step(error.gamma)
        self.calc_motors(xVel, yVel, gammaVel)
        self.set_motors()

    def path_step(self):
        self.target_step(updatePO=True)
        posError, rotError = (self.pathTargetPO - self.po).Abs2D()
        # print("pE: %.3f, rE: %.3f" % (posError, rotError))
        pathComplete = False
        if posError < 0.01 and rotError < 0.025:  # <2.5cm & <~1.25deg from path target
            pathComplete = True
        return pathComplete

    def path_goto(self, x, y, gamma):
        waypoint = copy.deepcopy(self.pathTargetPO.update())
        waypoint.x = x
        waypoint.y = y
        waypoint.gamma = gamma
        self.pathTargetPO.set(waypoint)
        self.tPrev = time.time()
        atDest = False
        while not atDest:
            tNow = time.time()
            if tNow - self.tPrev > self.tDuty:
                atDest = self.path_step()
