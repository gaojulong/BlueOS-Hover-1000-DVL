import os
import time

dat_path = "./2023-07-12 10-06.dat"

position_json = dict()
velocity_json = dict()
# data_json = dict()


def set_position(data_list):
    pass


def get_position():
    return position_json


def clear_position():
    position_json.clear()


def set_velocity(data):
    if data[0] == ":BI" and data[5] == "A":
        velocity_json['vx'] = float(data[1])
        velocity_json['vy'] = float(data[2])
        velocity_json['vz'] = float(data[3])


def get_velocity():
    return velocity_json


def clear_velocity():
    velocity_json.clear()


# 数据分流
def separating(data_line):
    data_list = data_line.replace(" ", "").replace("\n", "").split(",")
    # 系统姿态数据
    if data_list[0] == ":SA":
        set_position(data_line)

    # 大地坐标系下距离数据
    if data_list[0] == ":BD":
        set_position(data_line)

    # 设备坐标系下速度数据
    if data_list[0] == ":BI":
        set_velocity(data_line)


def main():
    with open(dat_path, 'r') as f:
        lines = f.readlines()
        for line in lines:
            separating(line)
            print(line)


if __name__ == '__main__':
    main()
