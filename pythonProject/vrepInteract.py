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
        return "Pos => X: %.3f,\t Y: %.3f,\t Z: %.3f\n" \
               "Rot => A: %.3f,\t B: %.3f,\t G: %.3f" \
               % (self.x, self.y, self.z,
                  self.alpha, self.beta, self.gamma)


def connect(_port=19999):
    vrep.simxFinish(-1)
    _client_id = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
    if _client_id != -1:
        print("Connection on port: {}, successful - ID: {}".format(_port, _client_id))
    else:
        print("Connection on port: {}, failed".format(_port))
    return _client_id


def get_handles():
    _handles = BotHandles()
    err_code, _handles.body = vrep.simxGetObjectHandle(clientId, "ME_Platfo2_sub1", vrep.simx_opmode_oneshot_wait)
    err_code, _handles.flM = vrep.simxGetObjectHandle(clientId, "rollingJoint_fl", vrep.simx_opmode_oneshot_wait)
    err_code, _handles.frM = vrep.simxGetObjectHandle(clientId, "rollingJoint_fr", vrep.simx_opmode_oneshot_wait)
    err_code, _handles.rlM = vrep.simxGetObjectHandle(clientId, "rollingJoint_rl", vrep.simx_opmode_oneshot_wait)
    err_code, _handles.rrM = vrep.simxGetObjectHandle(clientId, "rollingJoint_rr", vrep.simx_opmode_oneshot_wait)
    return _handles


def set_motors(_handles, bearing, spd):
    _speeds = MotorCommand()
    if bearing == 0:
        _speeds.fl = spd
        _speeds.fr = spd
        _speeds.rl = spd
        _speeds.rr = spd
    elif bearing == 90:
        _speeds.fl = spd
        _speeds.fr = -spd
        _speeds.rl = -spd
        _speeds.rr = spd
    elif bearing == 180:
        _speeds.fl = -spd
        _speeds.fr = -spd
        _speeds.rl = -spd
        _speeds.rr = -spd
    elif bearing == 270:
        _speeds.fl = -spd
        _speeds.fr = spd
        _speeds.rl = spd
        _speeds.rr = -spd

    vrep.simxSetJointTargetVelocity(clientId, _handles.flM, _speeds.fl, vrep.simx_opmode_oneshot_wait)
    vrep.simxSetJointTargetVelocity(clientId, _handles.frM, _speeds.fr, vrep.simx_opmode_oneshot_wait)
    vrep.simxSetJointTargetVelocity(clientId, _handles.rlM, _speeds.rl, vrep.simx_opmode_oneshot_wait)
    vrep.simxSetJointTargetVelocity(clientId, _handles.rrM, _speeds.rr, vrep.simx_opmode_oneshot_wait)


def get_pos_orien(_handles):
    _po = PosOrien()
    err_code, [_po.x, _po.y, _po.z] = vrep.simxGetObjectPosition(clientId, _handles.body, -1,
                                                                 vrep.simx_opmode_oneshot_wait)
    err_code, [_po.alpha, _po.beta, _po.gamma] = vrep.simxGetObjectOrientation(clientId, _handles.body, -1,
                                                                               vrep.simx_opmode_oneshot_wait)
    return _po


if __name__ == "__main__":
    clientId = connect()

    if clientId != -1:
        handles = get_handles()
        robPO = get_pos_orien(handles)
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
