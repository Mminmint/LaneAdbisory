# -*- coding: utf-8 -*-
# @Time    : 2024/11/27 20:17
# @Author  : Mminmint
# @File    : multiProcess.py
# @Software: PyCharm
import time

from simPredict import simExecute,multiSimExecute
import copy
import multiprocessing


# 只有一个用多进程更慢
def singleProcess(vehs,suggestLC):
    avgDist = simExecute(vehs, suggestLC)
    return [avgDist]


# 多进程
def multiProcess(processNum,vehs,suggestLCs):
    processes = []
    queue = multiprocessing.Queue()  # 创建一个队列用于收集结果

    # 使用多进程执行仿真
    for i in range(processNum):
        p = multiprocessing.Process(target=multiSimExecute, args=(vehs, suggestLCs[i],i,queue))
        processes.append(p)
        p.start()
        # print(time.time())

    # 等待所有进程完成
    for p in processes:
        p.join()

    # 收集所有仿真的返回值
    results = [queue.get() for _ in range(processNum)]
    return results


def processExecute(processNum,orgVehsInfo,suggestLCs):
    if processNum == 1:
        results = singleProcess(orgVehsInfo,suggestLCs[0])
    else:
        results = multiProcess(processNum,orgVehsInfo,suggestLCs)
    return results