import vrep
import time


def connect(port=19999):
    vrep.simxFinish(-1)
    client_id = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
    if client_id != -1:
        print("Connection on port: {}, successful - ID: ".format(port, client_id))
    else:
        print("Connection on port: {}, failed".format(port))
    return client_id


def get_motors():
    err_code, fl_motor_handle = vrep.simxGetObjectHandle(clientId, "rollingJoint_fl", vrep.simx_opmode_oneshot_wait)
    err_code, fr_motor_handle = vrep.simxGetObjectHandle(clientId, "rollingJoint_fr", vrep.simx_opmode_oneshot_wait)
    err_code, rl_motor_handle = vrep.simxGetObjectHandle(clientId, "rollingJoint_rl", vrep.simx_opmode_oneshot_wait)
    err_code, rr_motor_handle = vrep.simxGetObjectHandle(clientId, "rollingJoint_rr", vrep.simx_opmode_oneshot_wait)
    return fl_motor_handle, fr_motor_handle, rl_motor_handle, rr_motor_handle


def set_motors(handles, dir, spd):
    if dir is "forward":
        speeds = (spd, spd, spd, spd)
    elif dir is "backward":
        speeds = (-spd, -spd, -spd, -spd)
    elif dir is "left":
        speeds = (-spd, spd, spd, -spd)
    elif dir is "right":
        speeds = (spd, -spd, -spd, spd)

    vrep.simxSetJointTargetVelocity(clientId, handles[0], speeds[0], vrep.simx_opmode_oneshot_wait)
    vrep.simxSetJointTargetVelocity(clientId, handles[1], speeds[1], vrep.simx_opmode_oneshot_wait)
    vrep.simxSetJointTargetVelocity(clientId, handles[2], speeds[2], vrep.simx_opmode_oneshot_wait)
    vrep.simxSetJointTargetVelocity(clientId, handles[3], speeds[3], vrep.simx_opmode_oneshot_wait)


if __name__ == "__main__":
    clientId = connect()

    if clientId != -1:
        motor_handles = get_motors()

        speed = 1
        set_motors(motor_handles, "forward", speed)
        time.sleep(4)
        set_motors(motor_handles, "backward", speed)
        time.sleep(4)
        set_motors(motor_handles, "left", speed)
        time.sleep(4)
        set_motors(motor_handles, "right", speed)
        time.sleep(4)
        set_motors(motor_handles, "forward", 0)
