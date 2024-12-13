# -*- coding: utf-8 -*-
# @Time    : 2024/10/16 19:47
# @Author  : Mminmint
# @File    : simPredict.py
# @Software: PyCharm
import random
import time

import traci
import os, sys
import optparse
from vehicle import Vehicle
from toolFunction import startSUMO
from typing import Dict


'''
向路网中添加车辆，构造初始状态
'''
def addSimVehs(allVehs):
    traci.route.add("expressway", ["Input", "Output"])
    typeRef = {0: "HV", 1: "CV", 2: "CAV"}

    for veh in allVehs.values():
        traci.vehicle.add(veh.vehId, "expressway", typeID=typeRef[veh.type], depart='now',
                          departLane=veh.laneIndex, departPos=veh.position,departSpeed=veh.speed)
        if veh.LCModel is not None:
            traci.vehicle.setLaneChangeMode(veh.vehId, veh.LCModel)


'''
若车辆被建议换道成功，认为其接下来不会继续换道
禁用其换道模型
'''
def banLCModel(suggestLC):
    for vehID,laneID in suggestLC.items():
        if traci.vehicle.getLaneID(vehID) == 'Input_' + str(laneID):
            if laneID:
                traci.vehicle.setLaneChangeMode(vehID, 256)

'''
为符合条件的HV与驶出控制区后的optVehs执行静态晚合流控制
'''
def staticLateMerge():
    for vehID in traci.vehicle.getIDList():
        position = traci.vehicle.getLanePosition(vehID)
        if "hv" in vehID:
            if 1400 < position < 1500:
                traci.vehicle.setLaneChangeMode(vehID, 1621)
        else:
            if (2200 < position < 2300) and traci.vehicle.getLaneIndex(vehID):
                traci.vehicle.setLaneChangeMode(vehID, 1621)


'''
执行换道建议
suggestLC: {"cv.1":0,"cv.3":2...}
'''
def simLCExecute(suggestLC):
    for vehID,laneIndex in suggestLC.items():
        traci.vehicle.changeLane(vehID,laneIndex,6)


'''
所有车辆前进的总距离
'''
def forwardDist(allVehs) -> float:
    allDist = 0

    for vehID,veh in allVehs.items():
        # 若车辆还在路段内
        if vehID in traci.vehicle.getIDList():
            # 若车辆一开始在Input路段，后面驶入Output，position会突变
            if "Output" in traci.vehicle.getLaneID(vehID) and "Input" in veh.lane:
                dist = traci.vehicle.getLanePosition(vehID) + 2400
            else:
                dist = traci.vehicle.getLanePosition(vehID)
        # 若车辆已经驶出路段
        else:
            if "Output" in veh.lane:
                dist = 200
            else:
                dist = 2600
        allDist += (dist - veh.position)

    return allDist


'''
子仿真执行函数，最终输出车辆行驶的总距离
'''
def simExecute(allVehs,suggestLC) -> float:
    sumoCmd = startSUMO(False,"SubFile/SubTry.sumocfg")

    traci.start(sumoCmd, label="Sub")  # 打开仿真建立连接

    step = 0
    avgLCReactTime = 3      # todo: 灵敏度分析参数

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()

        if step == 0:
            addSimVehs(allVehs)
        else:
            # 到时间执行换道引导
            if step == avgLCReactTime:
                simLCExecute(suggestLC)
            # 到时间判断若换道引导成功执行，禁用自身换道模型
            if step == avgLCReactTime+10:
                banLCModel(suggestLC)
            # 为部分车辆实施静态晚合流
            staticLateMerge()

        step += 1
        if step == 60:
            allDist = forwardDist(allVehs)
            avgDist = allDist/(len(allVehs)*60)
            break

    traci.close()

    return avgDist


'''
子仿真多线程执行函数，最终输出车辆行驶的总距离
'''
def multiSimExecute(allVehs,suggestLC,simId,queue):
    print(f"process {simId} start in {time.time()}")

    sumoCmd = startSUMO(False, "SubFile/SubTry.sumocfg")

    traci.start(sumoCmd,label=f'Sub_{simId}')
    print(traci.getLabel())
    totalStep = 60
    avgLCReactTime = 3  # todo: 灵敏度分析参数

    for step in range(totalStep):
        traci.simulationStep()

        if step == 0:
            addSimVehs(allVehs)
        else:
            # 到时间执行换道引导
            if step == avgLCReactTime:
                simLCExecute(suggestLC)
            # 到时间判断若换道引导成功执行，禁用自身换道模型
            if step == avgLCReactTime + 10:
                banLCModel(suggestLC)
            # 为部分车辆实施静态晚合流
            staticLateMerge()

        step += 1
        if step == 60:
            allDist = forwardDist(allVehs)
            avgDist = allDist / (len(allVehs) * 60)
            break

    traci.close()

    # 将结果放入队列
    queue.put(avgDist)
    print(f"process {simId} end in {time.time()}")