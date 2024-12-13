# -*- coding: utf-8 -*-
# @Time    : 2024/10/14 15:51
# @Author  : Mminmint
# @File    : vehicles.py
# @Software: PyCharm
import copy
import random
import traci
from vehicle import Vehicle
from collections import defaultdict
from typing import Tuple


class Vehicles:

    def __init__(self):
        self.vehs = {}
        self.lastVehs = {}
        self.prepareLC = {}


    '''
    【main】
    遍历车辆，初始化vehs中的车辆信息
    获得optVehs
    ps: lastVehs和vehs引用的是一个对象，属性值改了会一起变
    '''
    def initVehs(self,step:int,curVehs:Tuple):
        self.step = step
        self.optVehs = {}

        for vehId in curVehs:
            # 初始化/更新车辆信息，添加至veh字典中
            if vehId not in self.lastVehs.keys():
                veh = self.addVeh(vehId)
            else:
                veh = self.lastVehs[vehId]
                veh.gainInfo()
                veh.updateLCInfo(self.step)
                self.vehs[vehId] = veh

            # 对车辆进行分类
            if not veh.type:
                veh.staticLateMerge()
            else:
                self.addOptVeh(veh)


    '''
    通过vehId新建veh类，初始化veh类属性
    将新出现车辆加入vehs中并返回当前车辆
    '''
    def addVeh(self,vehId:str) -> Vehicle:
        veh = Vehicle(vehId)
        veh.gainInfo()
        veh.initLCInfo()
        self.vehs[vehId] = veh

        return veh


    '''
    判断当前车辆是否在控制范围[500,2500]内
    在optVehs中加入
    为超过控制范围的车辆设置晚合流控制
    '''
    def addOptVeh(self,veh:Vehicle):
        if 200 < veh.position < 2200:
            self.optVehs[veh.vehId] = veh
        elif 2200 < veh.position < 2300:
            traci.vehicle.setLaneChangeMode(veh.vehId, 1621)
            veh.LCModel = 1621


    '''
    【main】
    将满足频率、安全的车辆依据车道给出
    readyLC: {0:["cv_0","cav_3"],1:["cv_1","cv_2","cav_2"]}
    '''
    def readyOptByLane(self):
        readyLC = defaultdict(list)

        for vehId,veh in self.optVehs.items():
            if veh.LCFrequency(self.step):
                if not veh.laneIndex:       # laneIndex为0
                    readyLCTag = veh.LCSafetyLeft(self.vehs)
                elif veh.laneIndex == 2:
                    readyLCTag = veh.LCSafetyRight(self.vehs)
                else:
                    readyLCTag = veh.LCSafetyLeft(self.vehs) and veh.LCSafetyRight(self.vehs)
                if readyLCTag:
                    readyLC[veh.laneIndex].append(vehId)

        return readyLC


    '''
    为当前时刻换道引导建议初始化车辆参数
    体现不确定参数：换道反应时间
    '''
    # todo:不确定性参数分布待确定
    def initLCs(self,suggestLC,avgReactTime,reactTimeBias):
        # 反应时间均匀分布
        downLimit = avgReactTime * reactTimeBias
        upLimit = avgReactTime * (2 - reactTimeBias)
        reactTime = round(random.uniform(downLimit, upLimit))

        for vehID,laneID in suggestLC.items():
            veh = self.vehs[vehID]
            veh.setLGInfo(self.step,'Input_'+str(laneID),reactTime)
            self.prepareLC[vehID] = self.vehs[vehID]


    '''
    为先前收到换道引导指令的车辆执行建议
    体现不确定参数：换道执行时间
    '''
    # todo:不确定性参数分布待确定
    def executeLCs(self,executeDuration,executeBias):
        tmpPrepareLC = copy.deepcopy(self.prepareLC)
        print(tmpPrepareLC)
        for vehID,veh in tmpPrepareLC.items():
            print(vehID,veh.LCReactTime)
            # 若是正在执行中，要获取随机参数duration
            if not veh.LCReactTime:
                # 换道执行时间均匀分布
                upLimit = executeDuration * (1 + executeBias)
                duration = round(random.uniform(3, upLimit))

                veh.LCExecute(isLC=1, duration=duration)
                del self.prepareLC[vehID]
            else:
                veh.LCExecute(isLC=0)


    def deinit(self):
        self.lastVehs = self.vehs
        self.vehs = {}