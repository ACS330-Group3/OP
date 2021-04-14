from VrepCustomBot import *

if __name__ == "__main__":
    bot = VrepBot()
    connected = bot.check_connected()

    print("Bot position:\n%s" % bot.po.update())
    bot.targetPO.update()
    print("Target position:\n%s" % bot.targetPO)
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

    waypoint = copy.deepcopy(bot.pathTargetPO.update())
    waypoints = list()
    for x, y in zip((1, 1, -1, -1), (1, -1, -1, 1)):
        waypoint.x = x
        waypoint.y = y
        waypoints.append(copy.deepcopy(waypoint))
    print(waypoints)
    i = 0
    repeats = 0
    while repeats < 2:
        bot.path_goto(waypoints[i].x, waypoints[i].y, waypoints[i].gamma)
        i = (i + 1) % 4
        if i == 0:
            repeats = repeats + 1

    bot.stop(True)  # force complete stop (blocking)
