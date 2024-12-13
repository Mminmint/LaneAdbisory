# -*- coding: utf-8 -*-
# @Time    : 2024/11/25 19:40
# @Author  : Mminmint
# @File    : multiProcessRef.py
# @Software: PyCharm

import multiprocessing
import traci
import time


def customize_vehicles(sim_id, num_vehicles):
    # 在第3秒个性化车辆命令
    for i in range(num_vehicles):
        veh_id = f"veh_{sim_id}_{i}"
        if veh_id in traci.vehicle.getIDList():
            traci.vehicle.setSpeed(veh_id, 10 + 2*sim_id)


def forwardDist(allVehs) -> float:
    allDist = 0
    for vehID in allVehs:
        allDist += traci.vehicle.getLanePosition(vehID)
    return allDist


def run_simulation(sim_id, num_vehicles, queue):
    print(f"Starting SIM {sim_id} ")
    sumo_cmd = [
        "sumo",  # 使用 sumo-gui 启动图形界面
        "-c", "SubFile/SubTry.sumocfg",  # 替换为您的 SUMO 配置文件
    ]

    # 启动 SUMO
    try:
        traci.start(sumo_cmd)
    except Exception as e:
        print(f"Failed to start SUMO simulation for SIM {sim_id}: {e}")
        queue.put(None)  # 将 None 放入队列以指示错误
        return

        # 添加车辆
    traci.route.add("expressway", ["Input", "Output"])
    for i in range(num_vehicles):
        veh_id = f"veh_{sim_id}_{i}"
        traci.vehicle.add(veh_id, "expressway", typeID="HV", depart='now',
                          departLane=0, departPos=0, departSpeed=10)  # 替换为您的线路 ID

    # 运行仿真
    total_steps = 50  # 设定仿真总步数
    for step in range(total_steps):
        traci.simulationStep()
        if step >= 3:
            customize_vehicles(sim_id, num_vehicles)  # 在第3秒执行个性化命令
        time.sleep(0.1)  # 在此等待以防止问题

    allVehs = traci.vehicle.getIDList()
    allDist = forwardDist(allVehs)

    # 关闭 TraCI
    traci.close()
    print(f"Finished SIM {sim_id}")

    # 将结果放入队列
    queue.put(allDist)

def single_run_simulation(sim_id, num_vehicles):
    print(f"Starting SIM {sim_id} ")
    sumo_cmd = [
        "sumo",  # 使用 sumo-gui 启动图形界面
        "-c", "SubFile/SubTry.sumocfg",  # 替换为您的 SUMO 配置文件
    ]

    # 启动 SUMO
    try:
        traci.start(sumo_cmd)
    except Exception as e:
        print(f"Failed to start SUMO simulation for SIM {sim_id}: {e}")
        return

        # 添加车辆
    traci.route.add("expressway", ["Input", "Output"])
    for i in range(num_vehicles):
        veh_id = f"veh_{sim_id}_{i}"
        traci.vehicle.add(veh_id, "expressway", typeID="HV", depart='now',
                          departLane=0, departPos=0, departSpeed=10)  # 替换为您的线路 ID

    # 运行仿真
    total_steps = 50  # 设定仿真总步数
    for step in range(total_steps):
        traci.simulationStep()
        if step >= 3:
            customize_vehicles(sim_id, num_vehicles)  # 在第3秒执行个性化命令
        time.sleep(0.1)  # 在此等待以防止问题

    allVehs = traci.vehicle.getIDList()
    allDist = forwardDist(allVehs)

    # 关闭 TraCI
    traci.close()
    print(f"Finished SIM {sim_id}")

    return allDist


def multi_main():
    num_simulations = 10  # 总仿真数量
    num_vehicles = 5  # 每个仿真中的车辆数量
    # ports = [9000 + i for i in range(num_simulations)]  # 更改为新的端口范围

    processes = []
    queue = multiprocessing.Queue()  # 创建一个队列用于收集结果

    # 使用多进程执行仿真
    for i in range(num_simulations):
        p = multiprocessing.Process(target=run_simulation, args=(i, num_vehicles, queue))
        processes.append(p)
        p.start()
        print(time.time())

    # 等待所有进程完成
    for p in processes:
        p.join()

    # 收集所有仿真的返回值
    results = [queue.get() for _ in range(num_simulations)]

    # 打印返回值
    for sim_id, result in enumerate(results):
        if result is not None:
            print(f"Simulation {sim_id} total distance: {result}")
        else:
            print(f"Simulation {sim_id} encountered an error.")


import concurrent.futures
import time

def multi_pool_main():
    num_simulations = 10  # 总仿真数量
    num_vehicles = 5  # 每个仿真中的车辆数量

    results = []

    # 使用进程池执行仿真
    with concurrent.futures.ProcessPoolExecutor() as executor:
        future_to_simulation = {executor.submit(run_simulation, i, num_vehicles): i for i in range(num_simulations)}

        for future in concurrent.futures.as_completed(future_to_simulation):
            sim_id = future_to_simulation[future]
            try:
                result = future.result()
                results.append(result)
                print(f"Simulation {sim_id} total distance: {result}")
            except Exception as e:
                print(f"Simulation {sim_id} encountered an error: {e}")


def single_main():
    num_simulations = 10  # 总仿真数量
    num_vehicles = 5  # 每个仿真中的车辆数量
    # ports = [9000 + i for i in range(num_simulations)]  # 更改为新的端口范围

    results = []
    # 使用多进程执行仿真
    for i in range(num_simulations):
        result = single_run_simulation(i, num_vehicles)
        results.append(result)

        time.sleep(0.5)  # 在启动每个进程之间增加延迟


    # 打印返回值
    for sim_id, result in enumerate(results):
        if result is not None:
            print(f"Simulation {sim_id} total distance: {result}")
        else:
            print(f"Simulation {sim_id} encountered an error.")


if __name__ == "__main__":
    # start = time.time()
    # multi_main()
    # end = time.time()
    # print("multi_pool:",end-start)

    sumo_cmd = [
        "sumo",  # 使用 sumo-gui 启动图形界面
        "-c", "MainFile/MainFile.sumocfg",  # 替换为您的 SUMO 配置文件
    ]
    traci.start(sumo_cmd,label="Main")

    step = 0


    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()

        if step >= 300:

            multi_main()

        # 操作结束，准备进入下一个步长
        step += 1
        if step == 1200:
            break

    traci.close()  # 关闭连接，还需在gui中点击退出并关闭gui
    # start = time.time()
    # single_main()
    # end = time.time()
    # print("single:",end-start)