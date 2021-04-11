import time
import copy
from VrepCustomBot import *

if __name__ == "__main__":
    bot = VrepBot()
    connected = bot.setup()

    print("Bot position:\n%s" % bot.get_pos_orien())
    err_code, targetHandle = vrep.simxGetObjectHandle(bot.clientId, "Target", vrep.simx_opmode_oneshot_wait)
    targetPO = PosOrien()
    targetPO.update(bot.clientId, targetHandle)
    print("Target position:\n%s" % targetPO)
    tPrev = time.time()
    tDuty = 0.1
    """
    duty cycle of 50ms
    calc and write motor speeds (blocking) takes ~30ms
        >>> before = time.time()
            bot.calc_motors(0, 0, 0.1)
            bot.set_motors(True)
            after = time.time()
            print("%.20f" % (after-before))
        0.03001093864440917969
    """
    pGain = 1
    while True:
        tNow = time.time()
        if tNow - tPrev > tDuty:
            # before = time.time()  # add to find fastest possible tDuty
            error = targetPO.update(bot.clientId, targetHandle) - bot.get_pos_orien()
            # print("Error:\n%s\n" % poError)  # add for Error printout
            xVel = pGain * error.x
            yVel = pGain * error.y
            w = pGain * error.gamma
            bot.calc_motors(xVel, yVel, w)
            bot.set_motors()
            tPrev = tPrev + tDuty
            # after = time.time()  # add to find fastest possible tDuty
            # print("%.20f" % (after - before))  # add to find fastest possible tDuty


    startBotPO = copy.deepcopy(bot.po)

    # # Travel test
    # prevBotPO = copy.deepcopy(bot.po)
    # speed = 0.25
    # for (x, y, w) in zip((speed, 0, -speed, 0), (0, speed, 0, -speed), (0, 0, 0, 0)):
    #     print("Executing: %.1f,\t %.1f,\t %.1f\n" % (x, y, w))
    #     bot.calc_motors(x, y, w)
    #     bot.set_motors()
    #     time.sleep(4)
    #     bot.stop(False)
    #     botPO = bot.get_pos_orien()
    #     print("gives delta:\n%s\n---   ---" % (botPO - prevBotPO))
    #     prevBotPO = copy.deepcopy(botPO)

    bot.stop(False)

    botPO = bot.get_pos_orien()
    print("Total travel:\n%s" % (botPO - startBotPO))

    bot.stop(True)  # force complete stop (blocking)
