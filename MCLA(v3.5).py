import traci
import math
import random
import numpy
import os

current_time = 0
simulation_end_time = 300

total_task = 0 # 產生的任務總數
task_type_counter = [0,0,0] # 紀錄各類型任務產生的數量
finished_task = 0 # 成功完成的任務總數

# 紀錄成功的任務中各類型任務的數量，list[0] -> 各類型成功數，list[1] -> 轎車完成的各類型數量，list[2] -> 公車完成的各類型數量，list[3] -> MEC完成的各類型數量
succ_recorder = [[0,0,0],[0,0,0],[0,0,0],[0,0,0]]

# 紀錄失敗的任務中各類型任務的數量
# list[0] -> 在task queue過期的各類型任務數, list[1] -> 卸載給公車失敗的各類型任務數, list[2] -> 卸載給MEC失敗的各類型任務數
# list[3] -> 在轎車完成運算前(排隊/執行中)過期的各類型任務數, list[4] -> 在公車完成運算前(排隊/執行中)過期的各類型任務數, list[5] -> 在MEC完成運算前(排隊/執行中)過期的各類型任務數
# list[6] -> 公車轉傳過程中過期的各類型任務數, list[7] -> 公車轉傳失敗的各類型任務數, list[8] -> MEC轉傳過程中過期的各類型任務數
# list[9] -> 公車回傳過程中過期的各類型任務數, list[10] -> 公車回傳失敗的各類型任務數, list[11] -> MEC回傳過程中過期的各類型任務數, list[12] -> MEC回傳失敗的各類型任務數
fail_recorder = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]

MEC_quantity = 20
MEC_location = [(250,250),(750,250),(1250,250),(1750,250),(2250,250),
                (250,750),(750,750),(1250,750),(1750,750),(2250,750),
                (250,1250),(750,1250),(1250,1250),(1750,1250),(2250,1250),
                (250,1750),(750,1750),(1250,1750),(1750,1750),(2250,1750)] # 每台固定式MEC伺服器的XY軸座標

# 紀錄各節點通訊範圍內其他的節點
car_nearby_buses = {}
car_nearby_MECs = {}
bus_nearby_cars = {}
bus_nearby_MECs = {}
MEC_nearby_cars = {}
MEC_nearby_buses = {}

car_wait_timer = {} # 記錄車輛可以生成任務的時間點
task_queue = {} # 存放車輛產生的任務
car_V2V_transmit_list = {} # 存放車輛正在傳送給公車的任務，存放的任務封包形式為包含兩個子list的list[[],[]]，第一個子list為task，第二個子list為[timestamp, 卸載目標, 傳輸延遲, 運算延遲, 完成時間]
car_V2I_transmit_list = {} # 存放車輛正在傳送給固定式MEC伺服器的任務，存放的任務封包形式同上

# 存放接收到的任務，轎車的queue中的任務封包形式為[[],[]]，第二個子list為[timestamp, 運算延遲, 完成時間], bus和MEC的queue中的任務封包形式則和car_transmit_list相同
car_receive_queue = {}
bus_receive_queue = {}
MEC_receive_queue = {}


# 存放正在執行的任務，轎車的queue中的任務封包形式與car_receive_queue相同，bus和MEC的queue中的任務封包形式和各自的receive_queue相同
car_process_list = {}
bus_process_list = {}
MEC_process_list = {}

# 存放正在回傳給轎車的任務，封包形式為[[], []]，第二個子list為[timestamp, 傳輸延遲]
bus_return_list = {}
MEC_return_list = {}

# 存放等待被轉傳的任務，封包形式為單純的task
bus_forward_list = {}
MEC_forward_list = {}

# 存放正在轉傳的任務，封包形式為[[], []]，第二個子list為[timestamp, 轉傳目標, 傳輸延遲]
bus_transmit_list = {}
MEC_transmit_list = {} 

# 各節點的運算能力
car_computation_capacity = 1000000000 # 1GHz
bus_computation_capacity = 2500000000 # 2.5GHz
MEC_computation_capacity = 5000000000 # 5GHz

CommRange = 250 # m
wired_network_throughput = 1000000000 # 1 Gbps
car_antenna_height = 1.7 # 公尺
bus_antenna_height = 3 # 公尺
MEC_antenna_height = 10 # 公尺

car_transmit_power = 23 #dBm
bus_transmit_power = 25 #dBm
MEC_transmit_power = 40 #dBm

car_transmit_antenna_gain = 3 #dB
car_receive_antenna_gain = 3 #dB
bus_transmit_antenna_gain = 3 #dB
bus_receive_antenna_gain = 3 #dB
MEC_transmit_antenna_gain = 10 #dB
MEC_receive_antenna_gain = 10 #dB

V2V_frequency_band = 5.9 # GHz
V2I_frequency_band = 3.5 # GHz

V2V_bandwidth = 20000000 # 20 MHz
V2I_bandwidth = 20000000 # 20 MHz
noise = 0.00000000008 # 8 * 10^-11 mW (計算公式為N = k * T * B)

V2V_shadow_fading_std = 3
V2I_shadow_fading_std = 4


def calculate_distance(node1_X, node1_Y, node2_X, node2_Y):
    return math.sqrt((node1_X - node2_X)**2 + (node1_Y - node2_Y)**2)


# 計算V2V-PC5的傳輸率
def get_PC5_transmissionRate(vehicle1, vehicle2):
    
    # 轎車->公車
    if vehicle1.startswith("car"):
        # 計算公車的接收功率
        path_loss = get_V2V_path_loss(vehicle1, vehicle2)
        shadow_fading = numpy.random.normal(0, V2V_shadow_fading_std)
        bus_receive_power = car_transmit_power + car_transmit_antenna_gain + bus_receive_antenna_gain - path_loss - shadow_fading
        brp_linear = 10 ** (bus_receive_power / 10) # 將接收功率(dB)轉為線性功率(mW)
        # 計算SINR
        Interference = 0  # 干擾功率的線性總和
        for car in bus_nearby_cars[vehicle2]:
            # 若該轎車正在進行V2V通訊(不包含傳遞timestamp為當前模擬時間的任務之轎車)，則視為干擾車輛
            if any(task_packet[1][0] < current_time for task_packet in car_V2V_transmit_list[car]):
                path_loss = get_V2V_path_loss(car, vehicle2)
                shadow_fading = numpy.random.normal(0, V2V_shadow_fading_std)
                receive_power = car_transmit_power + car_transmit_antenna_gain + bus_receive_antenna_gain - path_loss - shadow_fading
                rp_linear = 10 ** (receive_power / 10) # 將干擾功率(dB)轉為線性功率(mW)
                Interference += rp_linear
        SINR = brp_linear / (Interference + noise)
        # 計算傳輸率
        transmissionRate = V2V_bandwidth * math.log(1 + SINR, 2)
        
    # 公車->轎車
    elif vehicle1.startswith("bus"):
        # 計算轎車的接收功率
        path_loss = get_V2V_path_loss(vehicle1, vehicle2)
        shadow_fading = numpy.random.normal(0, V2V_shadow_fading_std)
        car_receive_power = bus_transmit_power + bus_transmit_antenna_gain + car_receive_antenna_gain - path_loss - shadow_fading
        crp_linear = 10 ** (car_receive_power / 10) # 將接收功率(dB)轉為線性功率(mW)
        # 計算SINR
        Interference = 0  # 干擾功率的線性總和
        for bus in car_nearby_buses[vehicle2]:
            # 若該公車為其他公車且正在進行V2V通訊(不包含傳遞timestamp為當前模擬時間的任務之公車)，則視為干擾車輛
            if bus != vehicle1 and any(task_packet[1][0] < current_time for task_packet in bus_return_list[bus]):
                path_loss = get_V2V_path_loss(bus, vehicle2)
                shadow_fading = numpy.random.normal(0, V2V_shadow_fading_std)
                receive_power = bus_transmit_power + bus_transmit_antenna_gain + car_receive_antenna_gain - path_loss - shadow_fading
                rp_linear = 10 ** (receive_power / 10) # 將干擾功率(dB)轉為線性功率
                Interference += rp_linear
        SINR = crp_linear / (Interference + noise)
        # 計算傳輸率
        transmissionRate = V2V_bandwidth * math.log(1 + SINR, 2)
    
    return transmissionRate


def get_V2V_path_loss(vehicle1, vehicle2):
    
    vehicle1_position = traci.vehicle.getPosition(vehicle1)
    vehicle2_position = traci.vehicle.getPosition(vehicle2)
    distance = calculate_distance(vehicle1_position[0], vehicle1_position[1], vehicle2_position[0], vehicle2_position[1])
    distance_3D = math.sqrt(distance**2 + abs(bus_antenna_height - car_antenna_height)**2)
    path_loss = 38.77 + 16.7 * math.log(distance_3D, 10) + 18.2 * math.log(V2V_frequency_band, 10)

    return path_loss


# 計算V2I-Uu link的傳輸率
def get_Uulink_transmissionRate(node1, node2):
    
    # 車輛->MEC
    if isinstance(node1, str):
        # 計算MEC的接收功率
        path_loss = get_V2I_path_loss(node1, node2)
        shadow_fading = numpy.random.normal(0, V2I_shadow_fading_std)
        # 轎車->MEC
        if node1.startswith("car"):
            MEC_receive_power = car_transmit_power + car_transmit_antenna_gain + MEC_receive_antenna_gain - path_loss - shadow_fading
        # 公車->MEC
        else:
            MEC_receive_power = bus_transmit_power + bus_transmit_antenna_gain + MEC_receive_antenna_gain - path_loss - shadow_fading
        mrp_linear = 10 ** (MEC_receive_power / 10) # 將接收功率(dB)轉為線性功率(mW)
        # 計算SINR
        Interference = 0  # 干擾功率的線性總和
        for car in MEC_nearby_cars[node2]:
            # 若該轎車正在進行V2I通訊(不包含傳遞timestamp為當前模擬時間的任務之轎車)，則視為干擾車輛
            if any(task_packet[1][0] < current_time for task_packet in car_V2I_transmit_list[car]):
                path_loss = get_V2I_path_loss(car, node2)
                shadow_fading = numpy.random.normal(0, V2I_shadow_fading_std)
                receive_power = car_transmit_power + car_transmit_antenna_gain + MEC_receive_antenna_gain - path_loss - shadow_fading
                rp_linear = 10 ** (receive_power / 10) # 將干擾功率(dB)轉為線性功率
                Interference += rp_linear
        for bus in MEC_nearby_buses[node2]:
            # 若該公車為其他車輛且正在進行V2I通訊(不包含傳遞timestamp為當前模擬時間的任務之公車)，則視為干擾車輛
            if bus != node1 and any(task_packet[1][0] < current_time for task_packet in bus_transmit_list[bus]):
                path_loss = get_V2I_path_loss(bus, node2)
                shadow_fading = numpy.random.normal(0, V2I_shadow_fading_std)
                receive_power = bus_transmit_power + bus_transmit_antenna_gain + MEC_receive_antenna_gain - path_loss - shadow_fading
                rp_linear = 10 ** (receive_power / 10) # 將干擾功率(dB)轉為線性功率
                Interference += rp_linear
        SINR = mrp_linear / (Interference + noise)
        # 計算傳輸率
        transmissionRate = V2I_bandwidth * math.log(1 + SINR, 2)
    
    # MEC->轎車
    else:
        # 計算轎車的接收功率
        path_loss = get_V2I_path_loss(node2, node1)
        shadow_fading = numpy.random.normal(0, V2I_shadow_fading_std)
        car_receive_power = MEC_transmit_power + MEC_transmit_antenna_gain + car_receive_antenna_gain - path_loss - shadow_fading
        mrp_linear = 10 ** (car_receive_power / 10) # 將接收功率(dB)轉為線性功率(mW)
        # 計算SINR
        Interference = 0  # 干擾功率的線性總和(因為環境假設基地台不重疊，因此不會有干擾的情況)
        SINR = mrp_linear / (Interference + noise)
        # 計算傳輸率
        transmissionRate = V2I_bandwidth * math.log(1 + SINR, 2)

    return transmissionRate


def get_V2I_path_loss(vehicle, MEC):

    if vehicle.startswith("car"):
        vehicle_antenna_height = car_antenna_height
    else:
        vehicle_antenna_height = bus_antenna_height
    h_UE = vehicle_antenna_height- 1
    h_BS = MEC_antenna_height - 1
    c = 3 * 10**8 # 光速
    d_BP = 4 * h_BS * h_UE * V2I_frequency_band * 10**9 / c
    vehicle_position = traci.vehicle.getPosition(vehicle)
    MEC_position = MEC_location[MEC]
    distance = calculate_distance(vehicle_position[0], vehicle_position[1], MEC_position[0], MEC_position[1])
    distance_3D = math.sqrt(distance**2 + abs(MEC_antenna_height - vehicle_antenna_height)**2)
    if distance <= d_BP:
        path_loss = 32.4 + 21 * math.log(distance_3D, 10) + 20 * math.log(V2I_frequency_band, 10)
    else:
        path_loss = 32.4 + 40 * math.log(distance_3D, 10) + 20 * math.log(V2I_frequency_band, 10) - 9.5 * math.log(d_BP**2 + abs(MEC_antenna_height - vehicle_antenna_height)**2, 10)
    
    return path_loss


# 更新各節點附近的其他類型節點
def update_nearby_nodes(cars_list, buses_list):

    car_nearby_buses.clear()
    car_nearby_MECs.clear()
    bus_nearby_cars.clear()
    bus_nearby_MECs.clear()
    MEC_nearby_cars.clear()
    MEC_nearby_buses.clear()

    # 紀錄轎車通訊範圍內的公車與固定式MEC伺服器
    for car in cars_list:
        car_position = traci.vehicle.getPosition(car)
        car_nearby_buses[car] = []
        for bus in buses_list:
            bus_position = traci.vehicle.getPosition(bus)
            distance = calculate_distance(car_position[0], car_position[1], bus_position[0], bus_position[1])
            if distance <= CommRange:
                car_nearby_buses[car].append(bus)
        car_nearby_MECs[car] = []
        for MEC in range(MEC_quantity):
            MEC_position = MEC_location[MEC]
            distance = calculate_distance(car_position[0], car_position[1], MEC_position[0], MEC_position[1])
            if distance <= CommRange:
                car_nearby_MECs[car].append(MEC)

    # 紀錄公車通訊範圍內的轎車與固定式MEC伺服器
    for bus in buses_list:
        bus_position = traci.vehicle.getPosition(bus)
        bus_nearby_cars[bus] = []
        for car in cars_list:
            car_position = traci.vehicle.getPosition(car)
            distance = calculate_distance(bus_position[0], bus_position[1], car_position[0], car_position[1])
            if distance <= CommRange:
                bus_nearby_cars[bus].append(car)
        bus_nearby_MECs[bus] = []
        for MEC in range(MEC_quantity):
            MEC_position = MEC_location[MEC]
            distance = calculate_distance(bus_position[0], bus_position[1], MEC_position[0], MEC_position[1])
            if distance <= CommRange:
                bus_nearby_MECs[bus].append(MEC)

    # 紀錄固定式MEC伺服器通訊範圍內的轎車與公車
    for MEC in range(MEC_quantity):
        MEC_position = MEC_location[MEC]
        MEC_nearby_cars[MEC] = []
        for car in cars_list:
            car_position = traci.vehicle.getPosition(car)
            distance = calculate_distance(MEC_position[0], MEC_position[1], car_position[0], car_position[1])
            if distance <= CommRange:
                MEC_nearby_cars[MEC].append(car)
        MEC_nearby_buses[MEC] = []
        for bus in buses_list:
            bus_position = traci.vehicle.getPosition(bus)
            distance = calculate_distance(MEC_position[0], MEC_position[1], bus_position[0], bus_position[1])
            if distance <= CommRange:
                MEC_nearby_buses[MEC].append(bus)


# 初始化各種存放任務的資料結構
def init_data_struct(cars_list, buses_list):

    for car in cars_list:
        if car not in task_queue:
            task_queue[car] = []
            car_V2V_transmit_list[car] = []
            car_V2I_transmit_list[car] = []
            car_receive_queue[car] = []
            car_process_list[car] = [] 
    
    for bus in buses_list:
        if bus not in bus_receive_queue:
            bus_receive_queue[bus] = []
            bus_process_list[bus] = []
            bus_return_list[bus] = []
            bus_forward_list[bus] = []
            bus_transmit_list[bus] = []

    for MEC in range(MEC_quantity):
        if MEC not in MEC_receive_queue:
            MEC_receive_queue[MEC] = []
            MEC_process_list[MEC] = []
            MEC_return_list[MEC] = []
            MEC_forward_list[MEC] = []
            MEC_transmit_list[MEC] = []


# 對已經離開路網的車輛做處理
def deal_left_car(cars_list):

    global total_task

    all_cars = set(task_queue.keys())
    left_cars = all_cars - set(cars_list)

    # 將與該車輛相關的所有任務移除，並調整total task的數量
    for car in left_cars:
        # task_queue
        total_task -= len(task_queue[car])
        del task_queue[car]
        # car_V2V_transmit_list
        total_task -= len(car_V2V_transmit_list[car])
        del car_V2V_transmit_list[car]
        # car_V2I_transmit_list
        total_task -= len(car_V2I_transmit_list[car])
        del car_V2I_transmit_list[car]
        # car_receive_queue
        total_task -= len(car_receive_queue[car])
        del car_receive_queue[car]
        # bus_receive_queue
        total_task -= sum(1 for task_packet_list in bus_receive_queue.values() for task_packet in task_packet_list if task_packet[0][0] == car)
        for bus in bus_receive_queue:
            bus_receive_queue[bus] = [task_packet for task_packet in bus_receive_queue[bus] if task_packet[0][0] != car]
        # MEC_receive_queue
        total_task -= sum(1 for task_packet_list in MEC_receive_queue.values() for task_packet in task_packet_list if task_packet[0][0] == car)
        for MEC in MEC_receive_queue:
            MEC_receive_queue[MEC] = [task_packet for task_packet in MEC_receive_queue[MEC] if task_packet[0][0] != car]
        # car_process_list
        total_task -= len(car_process_list[car])
        del car_process_list[car]
        # bus_process_list
        total_task -= sum(1 for task_packet_list in bus_process_list.values() for task_packet in task_packet_list if task_packet[0][0] == car)
        for bus in bus_process_list:
            bus_process_list[bus] = [task_packet for task_packet in bus_process_list[bus] if task_packet[0][0] != car]
        # MEC_process_list
        total_task -= sum(1 for task_packet_list in MEC_process_list.values() for task_packet in task_packet_list if task_packet[0][0] == car)
        for MEC in MEC_process_list:
            MEC_process_list[MEC] = [task_packet for task_packet in MEC_process_list[MEC] if task_packet[0][0] != car]
        # bus_return_list
        total_task -= sum(1 for task_packet_list in bus_return_list.values() for task_packet in task_packet_list if task_packet[0][0] == car)
        for bus in bus_return_list:
            bus_return_list[bus] = [task_packet for task_packet in bus_return_list[bus] if task_packet[0][0] != car]
        # MEC_return_list
        total_task -= sum(1 for task_packet_list in MEC_return_list.values() for task_packet in task_packet_list if task_packet[0][0] == car)
        for MEC in MEC_return_list:
            MEC_return_list[MEC] = [task_packet for task_packet in MEC_return_list[MEC] if task_packet[0][0] != car]
        # bus_forward_list
        total_task -= sum(1 for task_list in bus_forward_list.values() for task in task_list if task[0] == car)
        for bus in bus_forward_list:
            bus_forward_list[bus] = [task for task in bus_forward_list[bus] if task[0] != car]
        # MEC_forward_list
        total_task -= sum(1 for task_list in MEC_forward_list.values() for task in task_list if task[0] == car)
        for MEC in MEC_forward_list:
            MEC_forward_list[MEC] = [task for task in MEC_forward_list[MEC] if task[0] != car]
        # bus_transmit_list
        total_task -= sum(1 for task_packet_list in bus_transmit_list.values() for task_packet in task_packet_list if task_packet[0][0] == car)
        for bus in bus_transmit_list:
            bus_transmit_list[bus] = [task_packet for task_packet in bus_transmit_list[bus] if task_packet[0][0] != car]
        # MEC_transmit_list
        total_task -= sum(1 for task_packet_list in MEC_transmit_list.values() for task_packet in task_packet_list if task_packet[0][0] == car)
        for MEC in MEC_transmit_list:
            MEC_transmit_list[MEC] = [task_packet for task_packet in MEC_transmit_list[MEC] if task_packet[0][0] != car]


# 檢查執行的任務是否已經完成或過期
def check_process_task(cars_list, buses_list):

    global finished_task

    # check car
    for car in cars_list:
        # 檢查process list
        if car_process_list[car]:
            task_packet = car_process_list[car][0]
            # 如果任務過期或完成
            if (current_time >= task_packet[0][4] + task_packet[0][5]) or current_time >= task_packet[1][2]:
                # 將任務從process list移除
                car_process_list[car].pop(0)
                # 紀錄過期的任務
                if current_time >= task_packet[0][4] + task_packet[0][5]:
                    fail_recorder[3][task_packet[0][1]] += 1
                # 紀錄完成的任務
                else:
                    finished_task += 1
                    succ_recorder[0][task_packet[0][1]] += 1
                    succ_recorder[1][task_packet[0][1]] += 1

    # check bus
    for bus in buses_list:
        # 檢查process list
        if bus_process_list[bus]:
            task_packet = bus_process_list[bus][0]
            # 如果任務過期或完成
            if (current_time >= task_packet[0][4] + task_packet[0][5]) or current_time >= task_packet[1][4]:
                # 將任務從process list移除
                bus_process_list[bus].pop(0)
                # 紀錄過期的任務
                if current_time >= task_packet[0][4] + task_packet[0][5]:
                    fail_recorder[4][task_packet[0][1]] += 1
                # 若任務還沒過期且已經完成
                else:
                    # 卸載任務的車輛仍在公車的通訊範圍內 則放入回傳列表進行回傳
                    if task_packet[0][0] in bus_nearby_cars[bus]:
                        timestamp = current_time
                        PC5_transmissionRate = get_PC5_transmissionRate(bus, task_packet[0][0])
                        transmit_latency = task_packet[0][3] / PC5_transmissionRate
                        bus_return_list[bus].append([task_packet[0], [timestamp, transmit_latency]])
                    # 若卸載任務的車輛不在公車的通訊範圍內，則丟到轉傳列表等待處理
                    else:
                        bus_forward_list[bus].append(task_packet[0])

        
    # check MEC
    for MEC in range(MEC_quantity):
        # 檢查process list
        if MEC_process_list[MEC]:
            task_packet = MEC_process_list[MEC][0]
            # 如果任務過期或完成
            if (current_time >= task_packet[0][4] + task_packet[0][5]) or current_time >= task_packet[1][4]:
                # 將任務從process list移除
                MEC_process_list[MEC].pop(0)
                # 紀錄過期的任務
                if current_time >= task_packet[0][4] + task_packet[0][5]:
                    fail_recorder[5][task_packet[0][1]] += 1
                # 若任務還沒過期且已經完成
                else:
                    # 卸載任務的車輛仍在MEC伺服器的通訊範圍內 則放入回傳列表進行回傳
                    if task_packet[0][0] in MEC_nearby_cars[MEC]:
                        timestamp = current_time
                        Uulink_transmissionRate = get_Uulink_transmissionRate(MEC, task_packet[0][0])
                        transmit_latency = task_packet[0][3] / Uulink_transmissionRate
                        MEC_return_list[MEC].append([task_packet[0], [timestamp, transmit_latency]])
                    # 若卸載任務的車輛不在MEC伺服器的通訊範圍內，則丟到轉傳列表等待處理
                    else:
                        MEC_forward_list[MEC].append(task_packet[0])


# 轉傳完成的任務
def forward_task(buses_list):

    # 公車的部分
    for bus in buses_list:
        # 若有在某個固定式MEC伺服器的通訊範圍內
        if bus_nearby_MECs[bus]:
            for task in bus_forward_list[bus]:
                # 紀錄過期的任務
                if current_time >= task[4] + task[5]:
                    fail_recorder[6][task[1]] += 1
                # 若任務沒有過期，則將任務傳給該MEC伺服器
                else:
                    timestamp = current_time
                    Uulink_transmissionRate = get_Uulink_transmissionRate(bus, bus_nearby_MECs[bus][0])
                    transmit_latency = task[3] / Uulink_transmissionRate
                    bus_transmit_list[bus].append([task, [timestamp, bus_nearby_MECs[bus][0], transmit_latency]])
            # 若該時間步能和MEC伺服器通訊，則過期的任務會被移除，沒過期的任務會交給固定式MEC伺服器，因此轉傳列表會被清空
            bus_forward_list[bus].clear()
        # 若不在任何固定式MEC伺服器的通訊範圍內，就只將過期的任務移除
        else:
            bus_forward_list[bus] = [task for task in bus_forward_list[bus] if current_time < task[4] + task[5]]

        for task_packet in bus_transmit_list[bus]:
            # 紀錄過期的任務
            if current_time >= task_packet[0][4] + task_packet[0][5]:
                fail_recorder[6][task_packet[0][1]] += 1
            else:
                # 當任務沒過期且傳輸完成
                if (current_time >= task_packet[1][0] + task_packet[1][2]):
                    # 轉傳的MEC伺服器仍在通訊範圍內，則代表轉傳成功
                    if task_packet[1][1] in bus_nearby_MECs[bus]:
                        MEC_forward_list[task_packet[1][1]].append(task_packet[0])
                    # 轉傳失敗
                    else:
                        fail_recorder[7][task_packet[0][1]] += 1
        # 將傳輸完成和過期的任務移除
        bus_transmit_list[bus] = [task_packet for task_packet in bus_transmit_list[bus] if current_time < task_packet[0][4] + task_packet[0][5] and current_time < task_packet[1][0] + task_packet[1][2]]

    # MEC的部分
    # MEC成功轉傳給另一個MEC時是放入forward_list 因此要先檢查transmit list有沒有轉傳成功的任務，接著再檢查forward list中的任務能不能透過自己回傳給車輛，不行的話再丟給其他MEC
    for MEC in range(MEC_quantity):
        for task_packet in MEC_transmit_list[MEC]:
            # 紀錄過期的任務
            if current_time >= task_packet[0][4] + task_packet[0][5]:
                fail_recorder[8][task_packet[0][1]] += 1
            else:
                # 當任務沒過期且傳輸完成，則代表轉傳成功
                if current_time >= task_packet[1][0] + task_packet[1][2]:
                    MEC_forward_list[task_packet[1][1]].append(task_packet[0])
        # 將傳輸完成和過期的任務移除
        MEC_transmit_list[MEC] = [task_packet for task_packet in MEC_transmit_list[MEC] if current_time < task_packet[0][4] + task_packet[0][5] and current_time < task_packet[1][0] + task_packet[1][2]] 

        temp_list = [] # 用來保存還要繼續留在forward list中的任務
        for task in MEC_forward_list[MEC]:
            # 紀錄過期的任務
            if current_time >= task[4] + task[5]:
                fail_recorder[8][task[1]] += 1
            # 若任務沒有過期
            else:
                # 若轎車在任何一個固定式MEC伺服器的通訊範圍內
                if car_nearby_MECs[task[0]]:
                    # 若車輛在自己的通訊範圍內，則直接回傳給它
                    if task[0] in MEC_nearby_cars[MEC]:
                        timestamp = current_time
                        Uulink_transmissionRate = get_Uulink_transmissionRate(MEC, task[0])
                        transmit_latency = task[3] / Uulink_transmissionRate
                        MEC_return_list[MEC].append([task, [timestamp, transmit_latency]])
                    # 若不在自己的通訊範圍，則丟給能跟轎車通訊的MEC伺服器
                    else:
                        timestamp = current_time
                        transmit_latency = task[3] / wired_network_throughput
                        MEC_transmit_list[MEC].append([task, [timestamp, car_nearby_MECs[task[0]][0], transmit_latency]])
                # 保留還沒過期且尚未處理的任務
                else: 
                    temp_list.append(task)
        
        # 更新對應MEC的forward list
        MEC_forward_list[MEC] = temp_list


# 回傳完成的任務
def return_task():
    
    global finished_task

    # 公車的部分
    for bus, task_packet_list in bus_return_list.items():
        for task_packet in task_packet_list:
            # 紀錄過期的任務
            if current_time >= task_packet[0][4] + task_packet[0][5]:
                fail_recorder[9][task_packet[0][1]] += 1
            else:
                # 若任務已送達且沒有過期
                if current_time >= task_packet[1][0] + task_packet[1][1]:
                    # 若轎車仍在通訊範圍內，則代表任務成功
                    if task_packet[0][0] in bus_nearby_cars[bus]:
                        finished_task += 1
                        succ_recorder[0][task_packet[0][1]] += 1
                        succ_recorder[2][task_packet[0][1]] += 1
                    # 回傳失敗
                    else:
                        fail_recorder[10][task_packet[0][1]] += 1
        # 將傳輸完成和過期的任務移除
        bus_return_list[bus] = [task_packet for task_packet in bus_return_list[bus] if current_time < task_packet[0][4] + task_packet[0][5] and current_time < task_packet[1][0] + task_packet[1][1]]

    # MEC的部分
    for MEC, task_packet_list in MEC_return_list.items():
        for task_packet in task_packet_list:
            # 紀錄過期的任務
            if current_time >= task_packet[0][4] + task_packet[0][5]:
                fail_recorder[11][task_packet[0][1]] += 1
            else:
                # 若任務已送達且沒有過期
                if current_time >= task_packet[1][0] + task_packet[1][1]:
                    # 若轎車仍在通訊範圍內，則代表任務成功
                    if task_packet[0][0] in MEC_nearby_cars[MEC]:
                        finished_task += 1
                        succ_recorder[0][task_packet[0][1]] += 1
                        succ_recorder[3][task_packet[0][1]] += 1
                    # 回傳失敗
                    else:
                        fail_recorder[12][task_packet[0][1]] += 1
        # 將傳輸完成和過期的任務移除
        MEC_return_list[MEC] = [task_packet for task_packet in MEC_return_list[MEC] if current_time < task_packet[0][4] + task_packet[0][5] and current_time < task_packet[1][0] + task_packet[1][1]]


# 檢查哪些轎車能產生任務
def get_ready_list(cars_list):

    ready_car_list = []

    all_cars = set(car_wait_timer.keys())
    left_cars = all_cars - set(cars_list)

    # 將已經離開路網的轎車從等待列表移除
    for car_id in left_cars:
        del car_wait_timer[car_id]

    for car in cars_list:
        # 若車子是新加入的 則將它加入等待列表並開始一個等待時間
        if car not in car_wait_timer:
            car_wait_timer[car] = current_time + random.uniform(0.1, 2.0)
        else:
            # 若車子的等待時間結束 則準備產生任務  同時再開始一個等待時間
            if car_wait_timer[car] <= current_time:
                ready_car_list.append(car)
                car_wait_timer[car] = current_time + random.uniform(0.1, 2.0)
    
    return ready_car_list


# 將生成的任務放置對應車輛的task queue中
def generate_task(ready_car_list):

    global total_task

    for car in ready_car_list:
        
        task_amount = random.randint(3,7)
        for i in range(task_amount):
            rq_possibility = random.randint(1,100)
            task_level = random.randint(1,10)
            if 1 <= rq_possibility <= 40: # A類型任務 40%
                task_type = 0
                if 1 <= task_level <= 5: # 輕量級任務
                    require_cpu = random.randint(1, 2) * 10000000 # 所需CPU cycle
                else: # 中量級任務
                    require_cpu = random.randint(3, 4) * 10000000
                task_size = random.randint(100000, 1000000)
                tolerable_delay = 0.05  # 可接受延遲為50ms
                MEC_process_flag = random.choices([0, 1], weights=[90, 10])[0] # 是否必須由MEC伺服器處理 0 = no, 1 = yes
            elif 41 <= rq_possibility <= 80: # B類型任務 40%
                task_type = 1
                if 1 <= task_level <= 3: # 輕量級任務
                    require_cpu = random.randint(4, 5) * 10000000
                elif 4 <= task_level <= 8: # 中量級任務
                    require_cpu = random.randint(6, 7) * 10000000
                else: # 重量級任務
                    require_cpu = random.randint(11, 15) * 10000000
                task_size = random.randint(500000, 2500000)
                tolerable_delay = 0.15 # 可接受延遲為150ms
                MEC_process_flag = random.choices([0, 1], weights=[40, 60])[0]
            else: # C類型任務 20%
                task_type = 2
                if 1 <= task_level <= 3: # 輕量級任務
                    require_cpu = random.randint(8, 10) * 10000000
                elif 4 <= task_level <= 8: # 中量級任務
                    require_cpu = random.randint(12, 20) * 10000000
                else: # 重量級任務
                    require_cpu = random.randint(26, 37) * 10000000
                task_size = random.randint(800000, 5000000)
                tolerable_delay = 0.3 # 可接受延遲為300ms
                MEC_process_flag = random.choices([0, 1], weights=[10, 90])[0]

            task_generate_time = current_time
            task = [car, task_type, require_cpu, task_size, task_generate_time, tolerable_delay, MEC_process_flag]
            task_queue[car].append(task)
            total_task += 1
            task_type_counter[task_type] += 1


# 決定任務的卸載目標(自己/公車/MEC伺服器) 
def offload_decision():

    for car_id, task_list in task_queue.items():
        temp_list = [] # 用來儲存要繼續留在task_queue的task
        for task in task_list:
            if current_time >= task[4] + task[5]:  # 若任務已超時則丟棄
                fail_recorder[0][task[1]] += 1
                continue
            else:
                # 本地計算所要等的排隊時間(正在執行的任務之運算延遲+在queue中等待的任務之運算延遲)
                local_queue_delay = sum(math.ceil((task_packet[1][2] - current_time) * 100) / 100 for task_packet in car_process_list[car_id]) + sum(math.ceil(task_packet[1][1] * 100) / 100 for task_packet in car_receive_queue[car_id])
                # 滿足條件則本地計算(總延遲 < 剩餘時間 and 不用交給MEC計算)
                if (math.ceil(task[2] / car_computation_capacity * 100) / 100 + local_queue_delay < task[4] + task[5] - current_time) and task[6] == 0:
                    timestamp = current_time # 用來記錄發送此任務時當下的時間點
                    compute_latency = task[2] / car_computation_capacity
                    finish_time = 0 # 用來記錄完成此任務時的時間點(之後會被覆寫)
                    car_receive_queue[car_id].append([task,[timestamp, compute_latency, finish_time]])
                else:
                    candidate_buses = get_candidate_buses(task, car_nearby_buses[car_id])
                    # 若有可卸載的公車則卸載給延遲最低的公車
                    if candidate_buses:
                        min_delay_bus = min(candidate_buses, key=lambda bus: candidate_buses[bus][0] * 2 + candidate_buses[bus][1])
                        timestamp = current_time
                        transmit_latency = candidate_buses[min_delay_bus][0]
                        compute_latency = candidate_buses[min_delay_bus][1]
                        finish_time = 0
                        car_V2V_transmit_list[car_id].append([task, [timestamp, min_delay_bus, transmit_latency, compute_latency, finish_time]])
                    else:
                        candidate_MECs = get_candidate_MECs(task, car_nearby_MECs[car_id])
                        # 若有可卸載的MEC伺服器則卸載給延遲最低的MEC伺服器
                        if candidate_MECs:
                            min_delay_MEC = min(candidate_MECs, key=lambda MEC: candidate_MECs[MEC][0] * 2 + candidate_MECs[MEC][1])
                            timestamp = current_time
                            transmit_latency = candidate_MECs[min_delay_MEC][0]
                            compute_latency = candidate_MECs[min_delay_MEC][1]
                            finish_time = 0
                            car_V2I_transmit_list[car_id].append([task, [timestamp, min_delay_MEC, transmit_latency, compute_latency, finish_time]])
                        else:
                            temp_list.append(task)

        # 更新對應車輛的task queue
        task_queue[car_id] = temp_list
    

# 取得符合條件的公車候選人
def get_candidate_buses(task, nearby_buses_list):
    
    candidate_buses = {}

    for bus in nearby_buses_list: 
        predict_stayTime = BUS_get_predict_stayTime(task[0], bus)
        PC5_transmissionRate = get_PC5_transmissionRate(task[0], bus)
        transmission_delay = task[3] / PC5_transmissionRate
        computation_delay = task[2] / bus_computation_capacity
        # 公車計算所要等的排隊時間(正在執行的任務之運算延遲+在queue中等待的任務之運算延遲)
        bus_queue_delay = sum(math.ceil((task_packet[1][4] - current_time) * 100) / 100 for task_packet in bus_process_list[bus]) + sum(math.ceil(task_packet[1][3] * 100) / 100 for task_packet in bus_receive_queue[bus])
        total_delay = bus_queue_delay + 2 * math.ceil(transmission_delay * 100) / 100 + math.ceil(computation_delay * 100) / 100
        # 如果能在過期前完成且停留時間大於總延遲
        if total_delay < task[4] + task[5] - current_time and predict_stayTime > total_delay:
            candidate_buses[bus] = [transmission_delay, computation_delay]

    return candidate_buses


# 預測車輛停留在公車通訊範圍內的時間
def BUS_get_predict_stayTime(car, bus):

    car_position = traci.vehicle.getPosition(car)
    bus_position = traci.vehicle.getPosition(bus)
    car_speed = traci.vehicle.getSpeed(car)
    car_angle = math.radians(traci.vehicle.getAngle(car))
    bus_speed = traci.vehicle.getSpeed(bus)
    bus_angle = math.radians(traci.vehicle.getAngle(bus))

    # 計算相對初始位置
    delta_x = car_position[0] - bus_position[0]
    delta_y = car_position[1] - bus_position[1]

    # 計算相對速度
    v_x = car_speed * math.cos(car_angle) - bus_speed * math.cos(bus_angle)
    v_y = car_speed * math.sin(car_angle) - bus_speed * math.sin(bus_angle)

    distance = math.sqrt(CommRange**2 - delta_x**2) + math.sqrt(delta_y**2)

    # 相對速度為0，代表永遠不會離開，則設為最大值(60)
    if math.sqrt(v_x**2 + v_y**2) == 0:
        stay_time = 60
    # 若預測停留時間超過最大值(60)，則設為最大值
    else:
        stay_time =  min(distance / math.sqrt(v_x**2 + v_y**2), 60)

    return stay_time


# 取得符合條件的固定式MEC伺服器候選人
def get_candidate_MECs(task, car_nearby_MECs):

    candidate_MECs = {}

    for MEC in car_nearby_MECs: 
        predict_stayTime = MEC_get_predict_stayTime(task[0], MEC)
        Uulink_transmissionRate = get_Uulink_transmissionRate(task[0], MEC)
        transmission_delay = task[3] / Uulink_transmissionRate
        computation_delay = task[2] / MEC_computation_capacity
        # 計算所要等的排隊時間(正在執行的任務之運算延遲+在queue中等待的任務之運算延遲)
        MEC_queue_delay = sum(math.ceil((task_packet[1][4] - current_time) * 100) / 100 for task_packet in MEC_process_list[MEC]) + sum(math.ceil(task_packet[1][3] * 100) / 100 for task_packet in MEC_receive_queue[MEC])
        total_delay = MEC_queue_delay + 2 * math.ceil(transmission_delay * 100) / 100 + math.ceil(computation_delay * 100) / 100
        # 如果能在過期前完成且停留時間大於總延遲
        if total_delay < task[4] + task[5] - current_time and predict_stayTime > total_delay:
            candidate_MECs[MEC] = [transmission_delay, computation_delay]
    
    return candidate_MECs


# 預測車輛停留在固定式MEC通訊範圍內的時間
def MEC_get_predict_stayTime(car, mec_id):

    car_position = traci.vehicle.getPosition(car)
    car_speed = traci.vehicle.getSpeed(car)
    car_angle = math.radians(traci.vehicle.getAngle(car))

    # 計算相對初始位置
    delta_x = car_position[0] - MEC_location[mec_id][0]
    delta_y = car_position[1] - MEC_location[mec_id][1]

    vx = car_speed * math.cos(car_angle)
    vy = car_speed * math.sin(car_angle)

    # 透過圓的公式與判別式來計算預測時間

    A = vx**2 + vy**2
    B = 2 * (delta_x * vx + delta_y * vy)
    C = delta_x**2 + delta_y**2 - CommRange**2

    # 速度為0，代表永遠不會離開，則設為最大值(60)
    if A == 0:
        return 60

    D = B**2 - 4 * A * C
    
    # 若預測停留時間超過最大值(60)，則設為最大值
    predict_time = min((-B + math.sqrt(D)) / (2 * A), 60)

    return predict_time


# 確認任務是否成功卸載
def offload_task():
    
    #V2V
    for car, task_packet_list in car_V2V_transmit_list.items():
        for task_packet in task_packet_list:
            # 若完成卸載的時間點，卸載目標(公車)仍在車輛的通訊範圍內，則代表卸載成功
            if current_time >= task_packet[1][0] + task_packet[1][2]:
                if task_packet[1][1] in car_nearby_buses[car]:
                    bus_receive_queue[task_packet[1][1]].append(task_packet)
                # 卸載失敗
                else:
                    fail_recorder[1][task_packet[0][1]] += 1
        # 只保留仍在傳輸中的任務
        car_V2V_transmit_list[car] = [task_packet for task_packet in car_V2V_transmit_list[car] if current_time < task_packet[1][0] + task_packet[1][2]]

    #V2I
    for car, task_packet_list in car_V2I_transmit_list.items():
        for task_packet in task_packet_list:
            # 若完成卸載的時間點，卸載目標MEC伺服器)仍在車輛的通訊範圍內，則代表卸載成功
            if current_time >= task_packet[1][0] + task_packet[1][2]:
                if task_packet[1][1] in car_nearby_MECs[car]:
                    MEC_receive_queue[task_packet[1][1]].append(task_packet)
                # 卸載失敗
                else:
                    fail_recorder[2][task_packet[0][1]] += 1
        # 只保留仍在傳輸中的任務
        car_V2I_transmit_list[car] = [task_packet for task_packet in car_V2I_transmit_list[car] if current_time < task_packet[1][0] + task_packet[1][2]]


# 每個節點決定要執行哪些任務
def choose_task_to_process(cars_list, buses_list):

    # 車輛
    for car in cars_list:
        # 將過期的任務移除
        temp = [] # 儲存沒過期的任務
        for task_packet in car_receive_queue[car]:
            # 過期
            if current_time >= task_packet[0][4] + task_packet[0][5]:
                fail_recorder[3][task_packet[0][1]] += 1
            # 沒過期
            else:
                temp.append(task_packet)
        car_receive_queue[car] = temp
        # 根據timestamp來分類，timestamp小的排前面(模擬FIFO)
        car_receive_queue[car].sort(key=lambda x: x[1][0])

        # 若正在執行的任務已經完成，且待執行任務列表有任務，則執行第一個任務
        if not car_process_list[car] and car_receive_queue[car]:
            task_packet = car_receive_queue[car][0]
            # 將封包完成時間設為目前時間點 + 計算延遲
            task_packet[1][2] = current_time + task_packet[1][1]
            car_process_list[car].append(task_packet)
            car_receive_queue[car].pop(0)

    # 公車
    for bus in buses_list:
        # 將過期的任務移除
        temp = [] # 儲存沒過期的任務
        for task_packet in bus_receive_queue[bus]:
            # 過期
            if current_time >= task_packet[0][4] + task_packet[0][5]:
                fail_recorder[4][task_packet[0][1]] += 1
            # 沒過期
            else:
                temp.append(task_packet)
        bus_receive_queue[bus] = temp
        # 根據timestamp與傳輸延遲來分類，timestamp + 傳輸延遲之總和越小的排越前面(模擬FIFO)
        bus_receive_queue[bus].sort(key=lambda x: (x[1][0] + x[1][2]))

        # 若正在執行的任務已經完成，且待執行任務列表有任務，則執行第一個任務
        if not bus_process_list[bus] and bus_receive_queue[bus]:
            task_packet = bus_receive_queue[bus][0]
            # 將封包完成時間設為目前時間點 + 計算延遲
            task_packet[1][4] = current_time + task_packet[1][3]
            bus_process_list[bus].append(task_packet)
            bus_receive_queue[bus].pop(0)

    # MEC
    for MEC in range(MEC_quantity):
        # 將過期的任務移除
        temp = [] # 儲存沒過期的任務
        for task_packet in MEC_receive_queue[MEC]:
            # 過期
            if current_time >= task_packet[0][4] + task_packet[0][5]:
                fail_recorder[5][task_packet[0][1]] += 1
            # 沒過期
            else:
                temp.append(task_packet)
        MEC_receive_queue[MEC] = temp
        # 根據timestamp與傳輸延遲來分類，timestamp + 傳輸延遲之總和越小的排越前面(模擬FIFO)
        MEC_receive_queue[MEC].sort(key=lambda x: (x[1][0] + x[1][2]))

        # 若正在執行的任務已經完成，且待執行任務列表有任務，則執行第一個任務
        if not MEC_process_list[MEC] and MEC_receive_queue[MEC]:
            task_packet = MEC_receive_queue[MEC][0]
            # 將封包完成時間設為目前時間點 + 計算延遲
            task_packet[1][4] = current_time + task_packet[1][3]
            MEC_process_list[MEC].append(task_packet)
            MEC_receive_queue[MEC].pop(0)
    

def after_eval():

    print("total : ", total_task)
    print("finished : ", finished_task)
    print("total A : ", task_type_counter[0], ", total B : ", task_type_counter[1], " ,total C : ", task_type_counter[2])
    print("fin A : ", succ_recorder[0][0], ", fin B : ", succ_recorder[0][1], ", fin C : ", succ_recorder[0][2])
    print("car fin A : ", succ_recorder[1][0], ", car fin B : ", succ_recorder[1][1], ", car fin C : ", succ_recorder[1][2])
    print("bus fin A : ", succ_recorder[2][0], ", bus fin B : ", succ_recorder[2][1], ", bus fin C : ", succ_recorder[2][2])
    print("MEC fin A : ", succ_recorder[3][0], ", MEC fin B : ", succ_recorder[3][1], ", MEC fin C : ", succ_recorder[3][2])
    print("==========")
    print("task queue expired A : ", fail_recorder[0][0], ", task queue expired B : ", fail_recorder[0][1], ", task queue expired C : ", fail_recorder[0][2])
    print("bus offload failed A : ", fail_recorder[1][0], ", bus offload failed B : ", fail_recorder[1][1], ", bus offload failed C : ", fail_recorder[1][2])
    print("MEC offload failed A : ", fail_recorder[2][0], ", MEC offload failed B : ", fail_recorder[2][1], ", MEC offload failed C : ", fail_recorder[2][2])
    print("car receive expired A : ", fail_recorder[3][0], ", car receive expired B : ", fail_recorder[3][1], ", car receive expired C : ", fail_recorder[3][2])
    print("bus receive expired A : ", fail_recorder[4][0], ", bus receive expired B : ", fail_recorder[4][1], ", bus receive expired C : ", fail_recorder[4][2])
    print("MEC receive expired A : ", fail_recorder[5][0], ", MEC receive expired B : ", fail_recorder[5][1], ", MEC receive expired C : ", fail_recorder[5][2])
    print("bus foward expired A : ", fail_recorder[6][0], ", bus foward expired B : ", fail_recorder[6][1], ", bus foward expired C : ", fail_recorder[6][2])
    print("bus foward failed A : ", fail_recorder[7][0], ", bus foward failed B : ", fail_recorder[7][1], ", bus foward failed C : ", fail_recorder[7][2])
    print("MEC foward expired A : ", fail_recorder[8][0], ", MEC foward expired B : ", fail_recorder[8][1], ", MEC foward expired C : ", fail_recorder[8][2])
    print("bus return expired A : ", fail_recorder[9][0], ", bus return expired B : ", fail_recorder[9][1], ", bus return expired C : ", fail_recorder[9][2])
    print("bus return failed A : ", fail_recorder[10][0], ", bus return failed B : ", fail_recorder[10][1], ", bus return failed C : ", fail_recorder[10][2])
    print("MEC return expired A : ", fail_recorder[11][0], ", MEC return expired B : ", fail_recorder[11][1], ", MEC return expired C : ", fail_recorder[11][2])
    print("MEC return failed A : ", fail_recorder[12][0], ", MEC return failed B : ", fail_recorder[12][1], ", MEC return failed C : ", fail_recorder[12][2])
    print("----------")


def eval():
    a = 3
    b = 2
    c = 1
    print("finished rate : ", finished_task / total_task)
    print("weight finished rate : ", (a * succ_recorder[0][0] + b * succ_recorder[0][1] + c * succ_recorder[0][2]) / (a * task_type_counter[0] + b * task_type_counter[1] + c * task_type_counter[2]))


def main():

    global current_time

    base_path = os.path.dirname(os.path.abspath(__file__))
    sumo_config_path = os.path.join(base_path, "Mymethod.sumocfg")
    exe_mode = "sumo.exe"
    # exe_mode = "sumo_gui"

    traci.start([exe_mode, "-c", sumo_config_path, "--step-length", "0.01"])
    
    while traci.simulation.getMinExpectedNumber() > 0 and current_time <= simulation_end_time:
        
        traci.simulationStep()
        current_time = traci.simulation.getTime()

        print("time : ", current_time)

        vehicle_list = traci.vehicle.getIDList()
        cars_list = [v for v in vehicle_list if traci.vehicle.getTypeID(v) == "passenger"]
        buses_list = [v for v in vehicle_list if traci.vehicle.getTypeID(v) == "bus"]
        update_nearby_nodes(cars_list, buses_list)
        init_data_struct(cars_list, buses_list)
        deal_left_car(cars_list)
        check_process_task(cars_list, buses_list)
        forward_task(buses_list)
        return_task()
        ready_car_list = get_ready_list(cars_list)
        generate_task(ready_car_list)
        offload_decision()
        offload_task()
        choose_task_to_process(cars_list, buses_list)

        after_eval()

    traci.close()
    eval()

if __name__ == "__main__":
    main()