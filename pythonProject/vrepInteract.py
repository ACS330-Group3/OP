import time
import copy
from VrepCustomBot import *

if __name__ == "__main__":
    bot = VrepBot()
    connected = bot.setup()

    print("Bot position:\n%s" % bot.get_pos_orien())
    # err_code, targetHandle = vrep.simxGetObjectHandle(bot.clientId, "Target", vrep.simx_opmode_oneshot_wait)
    # targetPO = PosOrien()
    # targetPO.update(bot.clientId, targetHandle)
    # print("Target position:\n%s" % targetPO)
    # while True:
    #     print(targetPO)
    #     time.sleep(1)

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

    # botPO = get_pos_orien(handles)
    # print(botPO)
