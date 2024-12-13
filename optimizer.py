# -*- coding: utf-8 -*-
# @Time    : 2024/10/16 12:03
# @Author  : Mminmint
# @File    : optimizer.py
# @Software: PyCharm
import random
import numpy as np
import matplotlib.pyplot as plt
from copy import deepcopy
from multiProcess import processExecute
from operator import itemgetter
from typing import List,Dict
from matplotlib import rcParams

config = {
    "font.family": 'serif',
    "mathtext.fontset": 'stix',  # matplotlib渲染数学字体时使用的字体，和Times New Roman差别不大
    "font.serif": ['SimSun'],  # 宋体
    'axes.unicode_minus': False  # 处理负号，即-号
}
rcParams.update(config)


class Optimizer:
    def __init__(self,popNum,iterTimes,sameBestTimes,crossParam,mutationParam):
        self.popNum = popNum
        self.iterTimes = iterTimes
        self.sameBestTimes = sameBestTimes
        self.crossParam = crossParam
        self.mutationParam = mutationParam
        self.bestLC = None


    '''初始化种群'''
    def initPopulation(self):
        initPop = []

        for _ in range(self.popNum):
            # lane0: -1,1   lane1: 0,-1,2   lane2: 1,-1
            # 生成换道随机种子，不换道用-1表示
            newLC = []
            tmp = list(np.random.randint(0,2,self.LCBound[0]))
            newLC.extend(list(map(lambda x: -1 if x == 0 else x, tmp)))
            tmp = list(np.random.randint(0, 3, self.LCBound[1]-self.LCBound[0]))
            newLC.extend(list(map(lambda x: -1 if x == 1 else x, tmp)))
            tmp = list(np.random.randint(1, 3, self.LCBound[2]-self.LCBound[1]))
            newLC.extend(list(map(lambda x: -1 if x == 2 else x, tmp)))

            initPop.append({'LC': newLC, 'fit': -1})

        return initPop


    '''选择种群中的最优个体'''
    def selectBest(self,popWithFit:List[Dict]) -> Dict:
        sortPop = sorted(popWithFit,key=itemgetter("fit"),reverse=True)
        return sortPop[0]

    '''
    将便于变换的列表形式转换为真正需要换道的字典形式
    readyLC: ["cv_0","cav_3","cv_1","cv_2","cav_2"]
    pop["LC"]: [-1,1,-1,-1,2]
    
    suggestLC: {"cv.1": 0, "cv.3": 2}
    '''
    def transReadyToSuggest(self,individual:Dict):
        suggestLC = {}
        popLC = individual['LC']

        # 将需要变道的车辆加入suggest集合中
        for i in range(len(popLC)):
            if popLC[i] != -1:
                suggestLC[self.readyLC[i]] = popLC[i]

        return suggestLC


    '''计算一个种群的适应度'''
    def fitness(self,pop:List[Dict]) -> List[Dict]:
        waitFitness = []

        for i in range(len(pop)):
            if pop[i]['fit'] == -1:
                # 要进行一个readyLC和suggestLC的转换
                suggestLC = self.transReadyToSuggest(pop[i])
                waitFitness.append((i,suggestLC))

        # 有需要预测的适应度时
        if len(waitFitness):
            suggestLCs = [x[1] for x in waitFitness]
            results = processExecute(len(waitFitness),self.vehs,suggestLCs)

            # 和pop中的序号对上，赋值fit
            for i in range(len(waitFitness)):
                index = waitFitness[i][0]
                pop[index]['fit'] = results[i]

        return pop


    '''用轮盘赌方式按照概率从上一代选择个体直至形成新的一代'''
    def selection(self,popWithFit:List[Dict]) -> List[Dict]:
        afterSelect = []
        sumFit = sum(x['fit'] for x in popWithFit)
        sortPop = sorted(popWithFit,key=itemgetter("fit"), reverse=True)        # 从大到小排列

        for i in range(self.popNum):
            pointer = sumFit * np.random.uniform(0, 1)  # 随机产生一个[0,sum_fit]范围的数，即轮盘赌这局的指针
            curSum = 0
            for individual in sortPop:
                # 逐次累加从大到小排列的个体的适应度函数的值，直至超过指针，即选择它
                curSum += individual['fit']
                if curSum >= pointer:
                    afterSelect.append(sortPop[i])
                    break

        afterSelect = sorted(afterSelect,key=itemgetter('fit'), reverse=True)       # 从大到小排列选择的个体，方便进行交叉操作

        return afterSelect


    '''实现交叉操作'''
    def crossover(self,offSpring1,offSpring2):
        crossOff1, crossOff2 = {},{}

        # 交换换道区间
        pos1 = random.randrange(0, len(self.readyLC))
        pos2 = random.randrange(0, len(self.readyLC))
        if pos2 < pos1:
            pos1,pos2 = pos2,pos1
        crossOff1['LC'] = offSpring1['LC'][:pos1]+offSpring2['LC'][pos1:pos2]+offSpring1['LC'][pos2:]
        crossOff2['LC'] = offSpring2['LC'][:pos1]+offSpring1['LC'][pos1:pos2]+offSpring2['LC'][pos2:]

        crossOff1['fit'] = -1
        crossOff2['fit'] = -1

        return crossOff1,crossOff2


    '''实现变异操作'''
    def mutation(self,crossOff):
        # 选取变道变异点，依据可选择车道变异
        pos = random.randrange(0, len(self.readyLC))
        if self.LCBound[0] <= pos < self.LCBound[1]:
            choice = [-1,0,2]
            choice.remove(crossOff['LC'][pos])
            crossOff['LC'][pos] = random.choice(choice)
        else:
            crossOff['LC'][pos] = -crossOff['LC'][pos]

        crossOff['fit'] = -1

        return crossOff


    '''迭代图绘制'''
    def iterPlot(self,allFits):
        plt.figure(figsize=(4, 2))
        plt.plot(allFits)
        plt.xlim([1, 20])
        plt.xticks(range(1, self.popNum, 2))
        plt.xlabel('迭代次数', fontsize=12)
        plt.ylabel('平均行驶距离/m', fontsize=12)
        plt.show()


    '''优化主函数'''
    # todo:多进程
    def optimize(self,vehs,readyLC,LCBound):
        # 参数初始化
        print('-------------start-------------')
        self.vehs = vehs
        self.readyLC = readyLC
        self.LCBound = LCBound
        bestTimes = 1
        allFits,bestIndividuals = [],[]

        # 种群初始化
        # initPop: [{'LC': [-1,1,-1,-1,2,-1,1], 'fit': -1},
        #           {'LC': [-1,1,-1,-1,2,-1,1], 'fit': -1}]
        initPop = self.initPopulation()
        # 为初始化的种群计算fitness
        popWithFit = self.fitness(initPop)
        # 找到当前最优个体
        self.bestIndividual = self.selectBest(popWithFit)
        bestFit = self.bestIndividual['fit']
        allFits.append(bestFit)
        bestIndividuals.append(deepcopy(self.bestIndividual))

        for _ in range(self.iterTimes):
            # 选择、交叉及变异
            selectPop = self.selection(popWithFit)
            nextOff = []

            while len(nextOff) != self.popNum:
                offSpring1,offSpring2 = [selectPop.pop() for _ in range(2)]  # 后代间两两选择
                if random.random() < self.crossParam:
                    if len(self.readyLC) > 1:
                        crossOff1, crossOff2 = self.crossover(offSpring1,offSpring2)
                        if random.random() < self.mutationParam:
                            mutationOff1 = self.mutation(crossOff1)
                            mutationOff2 = self.mutation(crossOff2)
                            popWithFit = self.fitness([mutationOff1,mutationOff2])
                            nextOff.extend(popWithFit)
                        else:
                            popWithFit = self.fitness([crossOff1, crossOff2])
                            nextOff.extend(popWithFit)
                    else:
                        nextOff.extend([offSpring1, offSpring2])
                else:
                    nextOff.extend([offSpring1, offSpring2])

            popWithFit = nextOff
            bestIndividual = self.selectBest(popWithFit)
            curBestLC = bestIndividual['LC']
            curFit = bestIndividual['fit']

            # 更新最优算子
            if curFit > bestFit:
                self.bestIndividual = bestIndividual
                bestIndividuals.append(deepcopy(self.bestIndividual))
                bestFit = curFit
            allFits.append(bestFit)

            # 判断终止条件
            if self.bestIndividual['LC'] == curBestLC:
                bestTimes += 1
            else:
                bestTimes = 1

            if bestTimes >= self.sameBestTimes:
                break

        print(allFits)
        self.iterPlot(allFits)

        bestLC = self.transReadyToSuggest(self.bestIndividual)
        # self.bestLC = {'cv.0':1,'cv.5':1,'cav.2':1}

        return bestLC