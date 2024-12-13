# -*- coding: utf-8 -*-
# @Time    : 2024/11/28 19:21
# @Author  : Mminmint
# @File    : test.py
# @Software: PyCharm

from optimizer import Optimizer
from vehicles import Vehicles
from vehicle import Vehicle
import traci
import toolFunction


if __name__ == "__main__":
    sumoCmd = toolFunction.startSUMO(True, "MainFile/MainFile.sumocfg")
    traci.start(sumoCmd, label="Main")  # 打开仿真建立连接

    step = 0

    # 遗传算法参数
    popNum = 10
    iterTimes = 20
    sameBestTimes = 3
    crossParam = 0.6
    mutationParam = 0.1
    optimizer_ = Optimizer(popNum, iterTimes, sameBestTimes, crossParam, mutationParam)

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()

        if step >= 300:

            readyLC = ['cv.35','cv.22','cv.38','cv.23','cv.32']
            LCBound = [2, 4, 5]
            vehicles_ = Vehicles()

            veh1 = Vehicle('cv.22')
            veh1.acceleration = 0.0132345244632468
            veh1.lane = 'Input_1'
            veh1.laneIndex = 1
            veh1.position = 1934.0327118044786
            veh1.speed = 14.219149139476588
            veh1.type = 1

            veh2 = Vehicle('cv.23')
            veh2.acceleration = 0.009483792470392771
            veh2.lane = 'Input_2'
            veh2.laneIndex = 2
            veh2.position = 1986.8481294126082
            veh2.speed = 15.184215768317753
            veh2.type = 1

            veh3 = Vehicle('cv.35')
            veh3.acceleration = -0.08589025485091994
            veh3.lane = 'Input_0'
            veh3.laneIndex = 0
            veh3.position = 621.1533204801219
            veh3.speed = 16.798604601866305
            veh3.type = 1

            veh4 = Vehicle('cv.32')
            veh4.acceleration = 8.732671984112983e-05
            veh4.lane = 'Input_2'
            veh4.laneIndex = 2
            veh4.position = 923.7019361616128
            veh4.speed = 15.127865387805624
            veh4.type = 1

            veh5 = Vehicle('cv.38')
            veh5.acceleration = 0.00146845222332459
            veh5.lane = 'Input_1'
            veh5.laneIndex = 1
            veh5.position = 235.44011797156682
            veh5.speed = 15.36170843844695
            veh5.type = 1

            optVehs = {'cv.22':veh1,'cv.23':veh2,'cv.35':veh3,'cv.32':veh4,'cv.38':veh5}
            vehicles_.optVehs = optVehs
            vehicles_.vehs = optVehs

            # 算法优化结果
            suggestLC = optimizer_.optimize(vehicles_.vehs, readyLC, LCBound)

        # 操作结束，准备进入下一个步长
        step += 1
        if step == 1200:
            break

    traci.close()  # 关闭连接，还需在gui中点击退出并关闭gui