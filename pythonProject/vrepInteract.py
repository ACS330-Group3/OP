import vrep
import time


class MotorCommand:
    # Class/Struct of motor speeds
    def __init__(self):
        self.fl = 0
        self.fr = 0
        self.rl = 0
        self.rr = 0


class BotHandles:
    # Class/Struct of bot handles
    def __init__(self):
        self.body = 0
        self.flM = 0
        self.frM = 0
        self.rlM = 0
        self.rrM = 0


class PosOrien:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.alpha = 0
        self.beta = 0
        self.gamma = 0

    def __repr__(self):
        return "X: %.3f,\t Y: %.3f,\t Z: %.3f" % (self.x, self.y, self.z)


def connect(port=19999):
    vrep.simxFinish(-1)
    client_id = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
    if client_id != -1:
        print("Connection on port: {}, successful - ID: {}".format(port, client_id))
    else:
        print("Connection on port: {}, failed".format(port))
    return client_id


def get_handles():
    handles = BotHandles()
    err_code, handles.body = vrep.simxGetObjectHandle(clientId, "ME_Platfo2_sub1", vrep.simx_opmode_oneshot_wait)
    err_code, handles.flM = vrep.simxGetObjectHandle(clientId, "rollingJoint_fl", vrep.simx_opmode_oneshot_wait)
    err_code, handles.frM = vrep.simxGetObjectHandle(clientId, "rollingJoint_fr", vrep.simx_opmode_oneshot_wait)
    err_code, handles.rlM = vrep.simxGetObjectHandle(clientId, "rollingJoint_rl", vrep.simx_opmode_oneshot_wait)
    err_code, handles.rrM = vrep.simxGetObjectHandle(clientId, "rollingJoint_rr", vrep.simx_opmode_oneshot_wait)
    return handles


def set_motors(handles, bearing, spd):
    speeds = MotorCommand()
    if bearing == 0:
        speeds.fl = spd
        speeds.fr = spd
        speeds.rl = spd
        speeds.rr = spd
    elif bearing == 90:
        speeds.fl = spd
        speeds.fr = -spd
        speeds.rl = -spd
        speeds.rr = spd
    elif bearing == 180:
        speeds.fl = -spd
        speeds.fr = -spd
        speeds.rl = -spd
        speeds.rr = -spd
    elif bearing == 270:
        speeds.fl = -spd
        speeds.fr = spd
        speeds.rl = spd
        speeds.rr = -spd

    vrep.simxSetJointTargetVelocity(clientId, handles.flM, speeds.fl, vrep.simx_opmode_oneshot_wait)
    vrep.simxSetJointTargetVelocity(clientId, handles.frM, speeds.fr, vrep.simx_opmode_oneshot_wait)
    vrep.simxSetJointTargetVelocity(clientId, handles.rlM, speeds.rl, vrep.simx_opmode_oneshot_wait)
    vrep.simxSetJointTargetVelocity(clientId, handles.rrM, speeds.rr, vrep.simx_opmode_oneshot_wait)


if __name__ == "__main__":
    clientId = connect()

    if clientId != -1:
        handles = get_handles()
        robPO = PosOrien()

        err_code, [robPO.x, robPO.y, robPO.z] = vrep.simxGetObjectPosition(clientId, handles.body, -1, vrep.simx_opmode_oneshot_wait)
        print(robPO)

        speed = 1
        set_motors(handles, 0, speed)
        time.sleep(4)
        set_motors(handles, 90, speed)
        time.sleep(4)
        set_motors(handles, 180, speed)
        time.sleep(4)
        set_motors(handles, 270, speed)
        time.sleep(4)
        set_motors(handles, 0, 0)
