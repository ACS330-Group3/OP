import sys
import vrep
import time
import numpy as np

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


class VrepBot:
    def __init__(self):
        self.clientId = -1
        self.handles = BotHandles()
        self.speeds = MotorSpeeds()
        self.po = PosOrien()
        self.connect()
        self.get_handles()
        self.get_pos_orien()

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

    def get_handles(self):
        err_code, self.handles.body = vrep.simxGetObjectHandle(self.clientId, "ME_Platfo2_sub1", vrep.simx_opmode_oneshot_wait)
        err_code, self.handles.flM = vrep.simxGetObjectHandle(self.clientId, "rollingJoint_fl", vrep.simx_opmode_oneshot_wait)
        err_code, self.handles.frM = vrep.simxGetObjectHandle(self.clientId, "rollingJoint_fr", vrep.simx_opmode_oneshot_wait)
        err_code, self.handles.rlM = vrep.simxGetObjectHandle(self.clientId, "rollingJoint_rl", vrep.simx_opmode_oneshot_wait)
        err_code, self.handles.rrM = vrep.simxGetObjectHandle(self.clientId, "rollingJoint_rr", vrep.simx_opmode_oneshot_wait)
        return self.handles

    def calc_speeds_old(self, bearing, spd):
        _speeds = MotorSpeeds()
        if bearing == 0:
            self.speeds.fl = spd
            self.speeds.fr = spd
            self.speeds.rl = spd
            self.speeds.rr = spd
        elif bearing == 90:
            self.speeds.fl = spd
            self.speeds.fr = -spd
            self.speeds.rl = -spd
            self.speeds.rr = spd
        elif bearing == 180:
            self.speeds.fl = -spd
            self.speeds.fr = -spd
            self.speeds.rl = -spd
            self.speeds.rr = -spd
        elif bearing == 270:
            self.speeds.fl = -spd
            self.speeds.fr = spd
            self.speeds.rl = spd
            self.speeds.rr = -spd
        return self.speeds

    def calc_speeds(self, xSpd, ySpd, rotSpd):
        WHEEL_SEPARATION_WIDTH = 3.16
        WHEEL_SEPARATION_LENGTH = 4.56
        WHEEL_DIAMETER = 0.1
        revPerMeter = 1 / (WHEEL_DIAMETER * np.pi)

        self.speeds.fl = revPerMeter * (xSpd - ySpd - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*rotSpd)
        self.speeds.fr = revPerMeter * (xSpd + ySpd + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*rotSpd)
        self.speeds.rl = revPerMeter * (xSpd + ySpd - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*rotSpd)
        self.speeds.rr = revPerMeter * (xSpd - ySpd + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*rotSpd)
        return self.speeds

    def set_motors(self):
        vrep.simxSetJointTargetVelocity(self.clientId, self.handles.flM, self.speeds.fl, vrep.simx_opmode_oneshot_wait)
        vrep.simxSetJointTargetVelocity(self.clientId, self.handles.frM, self.speeds.fr, vrep.simx_opmode_oneshot_wait)
        vrep.simxSetJointTargetVelocity(self.clientId, self.handles.rlM, self.speeds.rl, vrep.simx_opmode_oneshot_wait)
        vrep.simxSetJointTargetVelocity(self.clientId, self.handles.rrM, self.speeds.rr, vrep.simx_opmode_oneshot_wait)

    def get_pos_orien(self):
        _po = PosOrien()
        err_code, [self.po.x, self.po.y, self.po.z] = vrep.simxGetObjectPosition(self.clientId, self.handles.body, -1, vrep.simx_opmode_oneshot_wait)
        err_code, [self.po.alpha, self.po.beta, self.po.gamma] = vrep.simxGetObjectOrientation(self.clientId, self.handles.body, -1, vrep.simx_opmode_oneshot_wait)
        return self.po

    def stop(self):
        self.speeds = MotorSpeeds()
        self.set_motors()


if __name__ == "__main__":
    bot = VrepBot()

    speed = 0.5
    for (x, y, w) in zip((speed, 0, -speed, 0), (0, speed, 0, -speed), (speed/10, -speed/10, speed/10, -speed/10)):
        print(x, y)
        tmpSpds = bot.calc_speeds(x, y, w)
        print(tmpSpds)
        bot.set_motors()
        time.sleep(2)

    bot.stop()

    botPO = bot.get_pos_orien()
    print(botPO)

    # botPO = get_pos_orien(handles)
    # print(botPO)
