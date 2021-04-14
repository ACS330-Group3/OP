import time
import copy
from VrepCustomBot import *

if __name__ == "__main__":
    bot = VrepBot()
    connected = bot.setup()

    print("Bot position:\n%s" % bot.get_pos_orien())
    err_code, targetHandle = vrep.simxGetObjectHandle(bot.clientId, "OmniTarget", vrep.simx_opmode_oneshot_wait)
    targetPO = PosOrien()
    targetPO.update(bot.clientId, targetHandle)
    print("Target position:\n%s" % targetPO)
    tPrev = time.time()
    tDuty = 0.1

    gains = PidGains()
    gains.p = 10
    gains.i = 0.5
    gains.d = 0.5
    bot.xPid.set_gains(copy.copy(gains))
    bot.yPid.set_gains(copy.copy(gains))
    bot.xPid.iLim = 0.1
    bot.yPid.iLim = 0.1
    # Uncomment for separate angular gains
    # gains.p = 10
    # gains.i = 0.5
    # gains.d = 0.5
    bot.gammaPid.set_gains(copy.copy(gains))
    bot.gammaPid.iLim = np.pi / 4

    while True:
        tNow = time.time()
        if tNow - tPrev > tDuty:
            bot.set_target(targetPO.update(bot.clientId, targetHandle))  # change for alternative target
            bot.get_pos_orien()
            bot.target_step()
            tPrev = tPrev + tDuty

    bot.stop(True)  # force complete stop (blocking)
