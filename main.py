# -*- coding: utf-8 -*-
# @Time    : 2024/10/14 22:04
# @Author  : Mminmint
# @File    : main.py
# @Software: PyCharm

from __future__ import absolute_import
from __future__ import print_function

import time

import traci
import os, sys
import optparse
import copy

import toolFunction
from vehicles import Vehicles
from vehicle import Vehicle
from optimizer import Optimizer


'''
将可换道的车辆字典转换为列表形式，并给出车道区分
readyLCDict: {0: ["cv_0", "cav_3"], 1: ["cv_1", "cv_2", "cav_2"]}
LCBound: [2,5]
readyLC: ["cv_0", "cav_3","cv_1", "cv_2", "cav_2"]
'''
def lineBound(readyLCDict):
    LCBound = [0]
    readyLC = []

    for i in range(3):
        if i in readyLCDict.keys():
            bound = LCBound[-1] + len(readyLCDict[i])
            LCBound.append(bound)
            readyLC.extend(readyLCDict[i])
        else:
            LCBound.append(LCBound[-1])

    LCBound.pop(0)
    return LCBound, readyLC


def run():
    sumoCmd = toolFunction.startSUMO(True, "MainFile/MainFile.sumocfg")
    traci.start(sumoCmd, label="Main")  # 打开仿真建立连接

    step = 0

    # 遗传算法参数
    popNum = 8
    iterTimes = 10
    sameBestTimes = 3
    crossParam = 0.6
    mutationParam = 0.1

    vehicles_ = Vehicles()
    optimizer_ = Optimizer(popNum,iterTimes,sameBestTimes,crossParam,mutationParam)
    start = time.time()

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()

        if step >= 300:
            curVehs = traci.vehicle.getIDList()

            # 初始化/更新vehs中的车辆信息，更新optVehs
            vehicles_.initVehs(step,curVehs)
            # 依据频率、安全约束给出可以接受变道/变速引导的车辆
            # readyLCDict: {0: ["cv_0", "cav_3"], 1: ["cv_1", "cv_2", "cav_2"]}
            # LCBound: [2,5]
            # readyLC: ["cv_0", "cav_3","cv_1", "cv_2", "cav_2"]
            readyLCDict = vehicles_.readyOptByLane()
            LCBound, readyLC = lineBound(readyLCDict)

            # todo: 如果解除这段注释，是可以并行的
            # veh1 = Vehicle('cv.22')
            # veh1.acceleration = 0.0132345244632468
            # veh1.lane = 'Input_1'
            # veh1.laneIndex = 1
            # veh1.position = 1934.0327118044786
            # veh1.speed = 14.219149139476588
            # veh1.type = 1
            #
            # veh2 = Vehicle('cv.23')
            # veh2.acceleration = 0.009483792470392771
            # veh2.lane = 'Input_2'
            # veh2.laneIndex = 2
            # veh2.position = 1986.8481294126082
            # veh2.speed = 15.184215768317753
            # veh2.type = 1
            #
            # veh3 = Vehicle('cv.35')
            # veh3.acceleration = -0.08589025485091994
            # veh3.lane = 'Input_0'
            # veh3.laneIndex = 0
            # veh3.position = 621.1533204801219
            # veh3.speed = 16.798604601866305
            # veh3.type = 1
            #
            # veh4 = Vehicle('cv.32')
            # veh4.acceleration = 8.732671984112983e-05
            # veh4.lane = 'Input_2'
            # veh4.laneIndex = 2
            # veh4.position = 923.7019361616128
            # veh4.speed = 15.127865387805624
            # veh4.type = 1
            #
            # veh5 = Vehicle('cv.38')
            # veh5.acceleration = 0.00146845222332459
            # veh5.lane = 'Input_1'
            # veh5.laneIndex = 1
            # veh5.position = 235.44011797156682
            # veh5.speed = 15.36170843844695
            # veh5.type = 1
            #
            # optVehs = {'cv.22': veh1, 'cv.23': veh2, 'cv.35': veh3, 'cv.32': veh4, 'cv.38': veh5}
            # vehicles_.optVehs = optVehs
            # vehicles_.vehs = optVehs

            # 算法优化结果
            suggestLC = optimizer_.optimize(vehicles_.vehs,readyLC,LCBound)

            # 主仿真执行之前发送的建议
            vehicles_.executeLCs(executeDuration=3,executeBias=0.8)
            # 主仿真发送当前优化的建议（待执行）
            vehicles_.initLCs(suggestLC,avgReactTime=3,reactTimeBias=0.8)

            # 更新vehs到lastVehs
            vehicles_.deinit()

        # 操作结束，准备进入下一个步长
        step += 1
        if step == 1200:
            break

    traci.close()  # 关闭连接，还需在gui中点击退出并关闭gui
    sys.stdout.flush()


if __name__ == "__main__":
    run()