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
            readyLCDict, readyLCCount = vehicles_.readyOptByLane()

            # 有需要优化的车辆才进入optimize
            if readyLCCount:
                # 将readyLCDict进行转化
                # readyLCDict: {0: ["cv_0", "cav_3"], 1: ["cv_1", "cv_2", "cav_2"]}
                # LCBound: [2,5]
                # readyLC: ["cv_0", "cav_3","cv_1", "cv_2", "cav_2"]
                LCBound, readyLC = lineBound(readyLCDict)
                # 整理车辆信息，便于多线程调用
                orgVehsInfo = vehicles_.organizeInfo()
                # 算法优化结果
                suggestLC = optimizer_.optimize(orgVehsInfo, readyLC, LCBound, readyLCCount)
            else:
                suggestLC = {}

            # 主仿真执行之前发送的建议
            vehicles_.executeLCs()

            if suggestLC:
                # 主仿真发送当前优化的建议（待执行）
                vehicles_.initLCs(suggestLC,avgReactTime=3,reactTimeBias=0.6)

            # 更新vehs到lastVehs
            vehicles_.deinit()

        # 操作结束，准备进入下一个步长
        step += 1
        if step == 1200:
            end = time.time()
            print("costTime:", end - start)
            break

    traci.close()  # 关闭连接，还需在gui中点击退出并关闭gui
    sys.stdout.flush()


if __name__ == "__main__":
    run()