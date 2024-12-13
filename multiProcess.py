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
        vehs_copy = copy.deepcopy(vehs)  # 确保每个进程有独立的车辆数据副本
        suggestLC_copy = suggestLCs[i]  # 获取当前进程的建议变道信息

        p = multiprocessing.Process(target=multiSimExecute, args=(vehs, suggestLC_copy,i,queue))
        processes.append(p)
        p.start()
        print(time.time())

    # 等待所有进程完成
    for p in processes:
        p.join()

    # 收集所有仿真的返回值
    results = [queue.get() for _ in range(processNum)]
    return results


def processExecute(processNum,vehs,suggestLCs):
    if processNum == 1:
        results = singleProcess(vehs,suggestLCs[0])
    else:
        results = multiProcess(processNum,vehs,suggestLCs)
    return results