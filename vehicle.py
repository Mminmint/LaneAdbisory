# -*- coding: utf-8 -*-
# @Time    : 2024/10/14 13:55
# @Author  : Mminmint
# @File    : vehicle.py
# @Software: PyCharm

import traci
from toolFunction import trimTuple


class Vehicle:

    def __init__(self,vehId):
        # 基本信息
        self.vehId = vehId          # str: "cv_1"
        self.laneIndex = None            # int: 0
        self.LCModel = None


    '''
    获取/更新车辆基本信息
    车辆类型、位置、速度、加速度
    Application: allVehs
    '''
    def gainInfo(self):
        # 车辆类型
        if self.vehId[:2] == 'hv':
            self.type = 0
        elif self.vehId[:2] == 'cv':
            self.type = 1
        else:
            self.type = 2

        self.position = traci.vehicle.getLanePosition(self.vehId)       # float: 50.236(m)
        self.speed = traci.vehicle.getSpeed(self.vehId)                 # float: 3.28(m/s)
        self.acceleration = traci.vehicle.getAcceleration(self.vehId)   # float: 0.28(m/s2)


    '''
    车辆刚进入车道时获取车道信息并初始化换道信息
    对于CV和CAV，初始化换道优化参数
    对于HV，设定换道模型
    Application: allVehs
    '''
    def initLCInfo(self):
        self.lane = traci.vehicle.getLaneID(self.vehId)
        self.laneIndex = int(self.lane[-1])     # int: 0

        if self.type:
            # 换道建议执行信息
            self.isSuggestLC = False
            self.targetLane = None      # str: "Input_1"
            self.LCReactTime = -1      # int
            self.lastLCTime = -1        # int
            self.lastSLCTime = -1       # int
            self.totalLCTimes = 0
        else:
            traci.vehicle.setLaneChangeMode(self.vehId, 0b011001010100)
            self.LCModel = 0b011001010100


    '''
    针对优化车辆更新车道信息以及换道时刻
    Application: optVehs
    '''
    def updateLCInfo(self,step:int):
        curLane = traci.vehicle.getLaneID(self.vehId)
        if self.type and curLane != self.lane:
            self.lastLCTime = step      # int
            self.totalLCTimes += 1
        self.lane = curLane
        self.laneIndex = int(self.lane[-1])


    '''
    为符合条件的HV执行静态晚合流控制
    Application: HVs
    '''
    def staticLateMerge(self):
        if 1400 < self.position < 1500:
            traci.vehicle.setLaneChangeMode(self.vehId, 1621)
            self.LCModel = 1621


    '''
    判断车辆向左换道的安全可行性
    获取左侧车道前后车信息判断间隙
    Application: optVehs
    '''
    # todo: 约束有效性，再看看模型
    def LCSafetyLeft(self,vehs):
        # traci给出的相对距离会减去miniGap
        vehIdLL, distLL = trimTuple(traci.vehicle.getNeighbors(self.vehId, 0b00000010))
        vehIdLF, distLF = trimTuple(traci.vehicle.getNeighbors(self.vehId, 0b00000000))

        self.vehLL = vehs.get(vehIdLL, None) if vehIdLL is not None else None
        self.vehLF = vehs.get(vehIdLF, None) if vehIdLF is not None else None
        distLL = distLL if distLL is not None else 300
        distLF = distLF if distLF is not None else 300
        speedLF = self.vehLF.speed if self.vehLF else 0     # 获取目标车道后车信息
        speedLL = self.vehLL.speed if self.vehLL else 25    # 获取目标车道前车信息

        t_ch = 3        # 变道时间
        t_act = 1       # 车辆感知到前车减速的反应时间
        h_min = 1       # tlv后端到tfv前端的车距最小要求
        a_min = -4.5    # 最大减速度

        # 如果不满足约束，则返回-1
        constrain1 = distLL + speedLF*t_ch + 0.5*a_min*t_ch*t_ch \
                     - self.speed*t_ch - 0.5*a_min*(t_ch-t_act)*(t_ch-t_act) - h_min
        if constrain1 < 0:
            return -1

        constrain2 = distLF + self.speed*t_ch + 0.5*a_min*t_ch*t_ch \
                     - speedLL*t_ch - 0.5*a_min*(t_ch-t_act)*(t_ch-t_act) - h_min

        return -1 if constrain2 <0 else 1


    '''
    判断车辆向右换道的安全可行性
    获取右侧车道前后车信息判断间隙
    Application: optVehs
    '''
    def LCSafetyRight(self,vehs):
        # traci给出的相对距离会减去miniGap
        vehIdOL, distOL = traci.vehicle.getLeader(self.vehId) if traci.vehicle.getLeader(self.vehId) is not None else (None,None)
        vehIdOF, distOF = traci.vehicle.getFollower(self.vehId) if traci.vehicle.getFollower(self.vehId) is not None else (None,None)
        vehIdRL, distRL = trimTuple(traci.vehicle.getNeighbors(self.vehId, 0b00000011))
        vehIdRF, distRF = trimTuple(traci.vehicle.getNeighbors(self.vehId, 0b00000001))

        distRL = distRL if distRL is not None else 300
        distRF = distRF if distRF is not None else 300

        self.vehOL = vehs.get(vehIdOL, None) if vehIdOL is not None else None
        self.vehOF = vehs.get(vehIdOF, None) if vehIdOF is not None else None
        self.vehRL = vehs.get(vehIdRL,None) if vehIdRL is not None else None
        self.vehRF = vehs.get(vehIdRF,None) if vehIdRF is not None else None

        t_ch = 3        # 变道时间
        t_act = 1       # 车辆感知到前车减速的反应时间
        h_min = 1       # tlv后端到tfv前端的车距最小要求
        a_min = -4.5    # 最大减速度

        speedRF = self.vehRF.speed if self.vehRF else 0     # 获取目标车道后车信息
        speedRL = self.vehRL.speed if self.vehRL else 25    # 获取目标车道前车信息

        # 如果不满足约束，则返回-1
        constrain1 = distRL + speedRF*t_ch + 0.5*a_min*t_ch*t_ch \
                     - self.speed*t_ch - 0.5*a_min*(t_ch-t_act)*(t_ch-t_act) - h_min
        if constrain1 < 0:
            return -1

        constrain2 = distRF + self.speed*t_ch + 0.5*a_min*t_ch*t_ch \
                     - speedRL*t_ch - 0.5*a_min*(t_ch-t_act)*(t_ch-t_act) - h_min
        return -1 if constrain2 <0 else 1


    # todo:这个self.LCModel在simPredict中用了吗
    '''
    换道引导频率约束
    包括与上一次换道、建议换道时间的间隔，总换道次数
    Application: optVehs
    '''
    def LCFrequency(self,step:int):
        # 距离上一次换道的时间（包括自主变道）
        if step - self.lastLCTime <= 10:
            return 0
        # 距离上一次收到换道建议的时间
        if step - self.lastSLCTime <= 20:
            return 0
        # 总换道次数过多且在开放车道，禁用换道模型
        if self.totalLCTimes >= 5 and self.laneIndex:
            traci.vehicle.setLaneChangeMode(self.vehId, 256)
            self.LCModel = 256
            return 0
        # 总换道次数过多且不在开放车道，采用静态晚合流
        if self.totalLCTimes >= 5 and not self.laneIndex:
            traci.vehicle.setLaneChangeMode(self.vehId,1621)
            self.LCModel = 1621
            return 0

        return 1


    '''
    设置换道引导参数
    当车辆在优化后接到速度引导后，需要改变参数记录
    '''
    def setLGInfo(self, step: int, targetLane: str, reactTime: int):
        self.lastSLCTime = step
        self.targetLane = targetLane
        self.LCReactTime = reactTime


    '''
    换道执行函数
    若反应时间到了，在规定duration内换道
    '''
    def LCExecute(self, isLC, duration=-1):
        if isLC:
            print("aaa")
            # duration:车辆会在这个时间内换道
            traci.vehicle.changeLane(self.vehId, int(self.targetLane[-1]), duration)
            self.isSuggestLC = False
            self.targetLane = None      # str: "Input_1"
            self.LCReactTime = -1       # int
        else:
            self.LCReactTime -= 1