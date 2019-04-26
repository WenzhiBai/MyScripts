#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
log visualiszation
Date:    2018/09/04 10:18:00
"""

import subprocess
import sys
import os
import math
import numpy as np
import matplotlib.pyplot as plt
import re
import time
import string
from datetime import datetime, timedelta
from dateutil.parser import parse

from numpy.random import rand

keywords = ["envmod msg:", "output traj:", "speed raw:", "gear:", "path_remain:", \
            "dis to end:", "Send cmd:", "current parkingstatetype:"]

trajectory_x = []
trajectory_y = []
trajectory_speed = []
obstacle_x = np.array([])
obstacle_y = np.array([])
single_obstacle_x = np.array([])
single_obstacle_r = np.array([])
parking_lot_x = [0.0, 0.0, 0.0, 0.0]
parking_lot_y = [0.0, 0.0, 0.0, 0.0]

vehicle_x = 0.0
vehicle_y = 0.0
vehicle_yaw = 0.0
vehicle_speed = 0.0
vehicle_gear = 0
dis_to_end = 0.0
path_remain = 0.0

parking_state = ""
send_cmd = ""

need_draw = False

def get_time(line):
    time_word = "1949-10-01 00:00:00.000"
    m = re.search("[0-9][0-9][0-9][0-9]-[0-9][0-9]-[0-9][0-9] [0-9][0-9]:[0-9][0-9]:[0-9][0-9].[0-9][0-9][0-9]", line)
    if m != None:
        time_word = m.group(0)
    return parse(time_word)

def get_trajectory(line):
    traj_indx = 0
    x_array = []
    y_array = []
    speed_array = []
    while traj_indx != -1:
        traj_indx = line.find("adc_trajectory_point", traj_indx)
        if traj_indx == -1:
            break
        x_indx = line.find("x:", traj_indx + 1)
        y_indx = line.find("y:", x_indx + 1)
        z_indx = line.find("z:", y_indx + 1)
        speed_indx = line.find("speed:", z_indx + 1)
        acceleration_s_indx = line.find("acceleration_s:", speed_indx + 1)
        x = string.atof(line[x_indx+2:y_indx])
        print(line[y_indx+2:z_indx])
        y = string.atof(line[y_indx+2:z_indx])
        speed = string.atof(line[speed_indx+6:acceleration_s_indx])
        x_array.append(x)
        y_array.append(y)
        speed_array.append(speed)
        traj_indx = y_indx
    if len(x_array) > 0:
        global need_draw
        global trajectory_x
        global trajectory_y
        global trajectory_speed
        need_draw = True
        trajectory_x = x_array
        trajectory_y = y_array
        trajectory_speed = speed_array
    return
# x_v: -3.0822606849670406 heading_c: 3.0740777452298866
def get_obstacle(line):
    obs_indx = 0
    x_array = []
    y_array = []
    obs_indx = line.find("k_objects")
    if obs_indx == -1:
        return
    words = line.split(" ")
    for i in range(len(words)-3):
        if words[i] == "x_v:" and words[i+2] == "heading_c:":
            x_v = string.atof(words[i+1])
            heading_c = string.atof(words[i+3])
            x_array.append(x_v)
            y_array.append(x_v * np.tan(heading_c))
    global need_draw
    global obstacle_x
    global obstacle_y
    need_draw = True
    obstacle_x = np.array(x_array)
    obstacle_y = np.array(y_array)
    return

def get_parking_lot(line):
    cur_indx = 0
    m = re.search("park [A-D] [x-z|X-Z]:", line[cur_indx:])
    global need_draw
    global parking_lot_x
    global parking_lot_y
    while m != None:
        cur_indx = cur_indx+m.start(0)
        #print(line[cur_indx+5])
        if line[cur_indx+5] == 'A':
            point_id = 1
        if line[cur_indx+5] == 'B':
            point_id = 2
        if line[cur_indx+5] == 'C':
            point_id = 3
        if line[cur_indx+5] == 'D':
            point_id = 0
        xyz_id = line[cur_indx+7]
        cur_indx = cur_indx+9
        #print(line[cur_indx:])
        m = re.search("park [A-D] [x-z|X-Z]:", line[cur_indx:])
        if m == None:
            break
        nxt_indx = m.start(0)
        #print(line[cur_indx+nxt_indx])
        #print(line[cur_indx:cur_indx+nxt_indx])
        val = string.atof(line[cur_indx:cur_indx+nxt_indx])
        if xyz_id == 'x':
            parking_lot_x[point_id] = val
        if xyz_id == 'y':
            parking_lot_y[point_id] = val
        #need_draw = True
    return

def get_vehicle(line):
    global need_draw
    global vehicle_x
    global vehicle_y
    global vehicle_yaw
    vehi_indx = line.find("VVV current_loc")
    if vehi_indx != -1:
        x_indx = line.find("x :", vehi_indx + 1)
        y_indx = line.find("y:", vehi_indx + 1)
        z_indx = line.find("heading:", vehi_indx + 1)
        #print(x_indx, y_indx, z_indx)
        #print(line[x_indx+3:y_indx])
        vehicle_x = string.atof(line[x_indx+3:y_indx])
        #print(line[y_indx+2:z_indx])
        vehicle_y = string.atof(line[y_indx+2:z_indx])
        vehicle_yaw = string.atof(line[z_indx+8:])
        need_draw = True
    return

def process_envmod_msg(line):
    envmod_indx = 0
    x_array = []
    y_array = []
    single_x_array = []
    single_r_array = []
    envmod_indx = line.find("envmod msg:")
    if envmod_indx == -1:
        return
    words = line[envmod_indx:].split(" ")
    global parking_lot_x
    global parking_lot_y
    global vehicle_x
    global vehicle_y
    global vehicle_yaw
    for i in range(len(words)):
        if i+3<len(words) and words[i] == "x_v:" and words[i+2] == "heading_c:":
            x_v = string.atof(words[i+1])
            heading_c = string.atof(words[i+3])
            x_array.append(x_v)
            y_array.append(x_v * np.tan(heading_c))
            i=i+3
        if i+5<len(words) and words[i] == "type:" and words[i+1] == "17" \
                          and words[i+2] == "x_v:" and words[i+4] == "heading_r:":
            x_v = string.atof(words[i+3])
            r = string.atoi(words[i+5])
            single_x_array.append(x_v)
            single_r_array.append(r)
            i=i+5
        if i+5<len(words) and words[i] == "pointA" and words[i+2] == "x:" and words[i+4] == "y:":
            parking_lot_x[1] = string.atof(words[i+3])
            parking_lot_y[1] = string.atof(words[i+5])
            i=i+5
        if i+5<len(words) and words[i] == "pointB" and words[i+2] == "x:" and words[i+4] == "y:":
            parking_lot_x[2] = string.atof(words[i+3])
            parking_lot_y[2] = string.atof(words[i+5])
            i=i+5
        if i+5<len(words) and words[i] == "pointC" and words[i+2] == "x:" and words[i+4] == "y:":
            parking_lot_x[3] = string.atof(words[i+3])
            parking_lot_y[3] = string.atof(words[i+5])
            i=i+5
        if i+5<len(words) and words[i] == "pointD" and words[i+2] == "x:" and words[i+4] == "y:":
            parking_lot_x[0] = string.atof(words[i+3])
            parking_lot_y[0] = string.atof(words[i+5])
            i=i+5
        if i+5<len(words) and words[i] == "position" and words[i+2] == "x:" and words[i+4] == "y:":
            vehicle_x = string.atof(words[i+3])
            vehicle_y = string.atof(words[i+5])
            i=i+5
        if i+9<len(words) and words[i] == "orientation" and words[i+2] == "x:" and words[i+4] == "y:":
            qx = string.atof(words[i+3])
            qy = string.atof(words[i+5])
            qz = string.atof(words[i+7])
            qw = string.atof(words[i+9])
            vehicle_yaw = np.arctan2((2 * qx * qy + 2 * qw * qz), (1 - 2 * qy * qy - 2 * qz * qz))
            i=i+9
    global need_draw
    global obstacle_x
    global obstacle_y
    global single_obstacle_x
    global single_obstacle_r
    need_draw = True
    obstacle_x = np.array(x_array)
    obstacle_y = np.array(y_array)
    single_obstacle_x = np.array(single_x_array)
    single_obstacle_r = np.array(single_r_array)
    return

def distance(x1, x2, y1, y2):
    dis = np.sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2))
    return dis

def draw_all(cur_tm):
    plt.cla()
    plt.xlabel("X axie")
    global parking_lot_x
    global parking_lot_y
    rec_x = -100
    for x in range(-100, 100, 10):
        if parking_lot_x[0] < x:
            rec_x = x - 15.0
            break
    plt.xlim(rec_x, rec_x+30.0)
    plt.ylabel("Y axie")
    plt.ylim(-10.0, 10.0)
    global trajectory_x
    global trajectory_y
    global trajectory_speed

    plt.plot(trajectory_x, trajectory_y)
    plt.plot(parking_lot_x, parking_lot_y)

    lane_width = 5.0
    extended_left_length = 1.85
    extended_right_length = 4.85
    theta = np.arctan2((parking_lot_y[0] - parking_lot_y[1]),
                       (parking_lot_x[0] - parking_lot_x[1]))
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    E_x = parking_lot_x[0] + lane_width * cos_theta - extended_left_length * sin_theta
    E_y = parking_lot_y[0] + lane_width * sin_theta + extended_left_length * cos_theta

    F_x = parking_lot_x[3] + lane_width * cos_theta + extended_right_length * sin_theta
    F_y = parking_lot_y[3] + lane_width * sin_theta - extended_right_length * cos_theta

    lane_x = [E_x, F_x]
    lane_y = [E_y, F_y]
    plt.plot(lane_x, lane_y)

    global vehicle_x
    global vehicle_y
    global vehicle_yaw
    global vehicle_gear

    cos_vehicle_yaw = np.cos(vehicle_yaw)
    sin_vehicle_yaw = np.sin(vehicle_yaw)
    car_x = []
    car_y = []
    vehicle_rear_to_bottom = 1.03
    vehicle_rear_to_top = 3.7
    vehicle_width_divide2 = 1.77/2.0

    car_x.append(-vehicle_rear_to_bottom * cos_vehicle_yaw - vehicle_width_divide2 * sin_vehicle_yaw + vehicle_x)
    car_x.append(-vehicle_rear_to_bottom * cos_vehicle_yaw + vehicle_width_divide2 * sin_vehicle_yaw + vehicle_x)
    car_x.append(vehicle_rear_to_top * cos_vehicle_yaw + vehicle_width_divide2 * sin_vehicle_yaw + vehicle_x)
    car_x.append(vehicle_rear_to_top * cos_vehicle_yaw - vehicle_width_divide2 * sin_vehicle_yaw + vehicle_x)
    car_x.append(-vehicle_rear_to_bottom * cos_vehicle_yaw - vehicle_width_divide2 * sin_vehicle_yaw + vehicle_x)

    car_y.append(-vehicle_rear_to_bottom * sin_vehicle_yaw + vehicle_width_divide2 * cos_vehicle_yaw + vehicle_y)
    car_y.append(-vehicle_rear_to_bottom * sin_vehicle_yaw - vehicle_width_divide2 * cos_vehicle_yaw + vehicle_y)
    car_y.append(vehicle_rear_to_top * sin_vehicle_yaw - vehicle_width_divide2 * cos_vehicle_yaw + vehicle_y)
    car_y.append(vehicle_rear_to_top * sin_vehicle_yaw + vehicle_width_divide2 * cos_vehicle_yaw + vehicle_y)
    car_y.append(-vehicle_rear_to_bottom * sin_vehicle_yaw + vehicle_width_divide2 * cos_vehicle_yaw + vehicle_y)

    plt.plot(car_x, car_y)
    plt.scatter(vehicle_x, vehicle_y, s=3, c='r', marker = 'o')

    global obstacle_x
    global obstacle_y
    obstacle_x_v = vehicle_x + obstacle_x * np.cos(vehicle_yaw) - obstacle_y * np.sin(vehicle_yaw)
    obstacle_y_v = vehicle_y + obstacle_x * np.sin(vehicle_yaw) + obstacle_y * np.cos(vehicle_yaw)
    plt.scatter(obstacle_x_v, obstacle_y_v, marker = 'o')
    for i in range(len(obstacle_x_v)):
        dis = distance(vehicle_x, obstacle_x_v[i], vehicle_y, obstacle_y_v[i])
        plt.text(obstacle_x_v[i], obstacle_y_v[i], dis)
    global single_obstacle_x
    global single_obstacle_r
    for i in range(len(single_obstacle_x)):
        if single_obstacle_r[i] == 1 or single_obstacle_r[i] == 2 or single_obstacle_r[i] == 3:
            plt.scatter(car_x[2], car_y[2], marker = 'o')
            plt.text(car_x[2], car_y[2], single_obstacle_x[i])
        if single_obstacle_r[i] == 4 or single_obstacle_r[i] == 5 or single_obstacle_r[i] == 6:
            plt.scatter(car_x[3], car_y[3], marker = 'o')
            plt.text(car_x[3], car_y[3], single_obstacle_x[i])
        if single_obstacle_r[i] == 7 or single_obstacle_r[i] == 8 or single_obstacle_r[i] == 9:
            plt.scatter(car_x[0], car_y[0], marker = 'o')
            plt.text(car_x[0], car_y[0], single_obstacle_x[i])
        if single_obstacle_r[i] == 10 or single_obstacle_r[i] == 11 or single_obstacle_r[i] == 12:
            plt.scatter(car_x[1], car_y[1], marker = 'o')
            plt.text(car_x[1], car_y[1], single_obstacle_x[i])
    for i in range(len(trajectory_x)):
        plt.text(trajectory_x[i], trajectory_y[0]+i*0.6, "speed:"+"{:.3f}".format(trajectory_speed[i]))
    global dis_to_end
    global send_cmd
    global parking_state
    global path_remain
    text_show = "{:.3f}".format(parking_lot_x[1]) + ", " +  "{:.3f}".format(parking_lot_y[1])
    plt.text(parking_lot_x[1], parking_lot_y[1], text_show)
    plt.text(rec_x+0.5, -6.5, "dis to end:"+"{:.3f}".format(dis_to_end))
    plt.text(rec_x+0.5, -7.5, "gear:"+str(vehicle_gear))
    plt.text(rec_x+0.5, -8.5, "speed raw:"+"{:.3f}".format(vehicle_speed))
    plt.text(rec_x+0.5, -9.5, "time:"+str(cur_tm))
    plt.text(rec_x+0.5, 9.5, "cmd:"+send_cmd)
    plt.text(rec_x+0.5, 8.5, "path remain:"+"{:.3f}".format(path_remain))
    plt.text(rec_x+0.5, 7.5, "parkingstate:"+parking_state)
    plt.pause(0.05)

def process_dict(line_dict):
    cur_tm = datetime(1970, 1, 1, 0, 0, 0)
    plt.figure(figsize=(9, 6))
    plt.ion()
    global need_draw
    global vehicle_speed
    global vehicle_gear
    global dis_to_end
    global path_remain
    global send_cmd
    global parking_state
    global trajectory_x
    global trajectory_y
    first_run = True
    for tm, line in sorted(line_dict.iteritems()):
        if tm < cur_tm + timedelta(0, -1):
            if first_run == False:
                cur_tm = datetime(1970, 1, 1, 0, 0, 1)
                #first_run = True
            else:
                continue
        elif first_run:
            cur_tm = tm + timedelta(0, delta_sec)
            first_run = False
        if tm >= cur_tm + timedelta(0, 0.1):
            cur_tm = tm + timedelta(0, 0.1)
            print(cur_tm)
            if need_draw and len(trajectory_x) > 0:
                draw_all(cur_tm)
            need_draw = False
        if line.find("speed raw:") != -1:
            vehicle_speed = string.atof(line.split(" ")[-1])
        if line.find("dis to end:") != -1:
            dis_to_end = string.atof(line.split(" ")[-1])
        if line.find("path_remain") != -1:
            path_remain = string.atof(line.split(" ")[-1])
        if line.find("Send cmd:") != -1:
            send_cmd = line[line.find("Send cmd:")+10:].strip()
        if line.find("current parkingstatetype:") != -1:
            indx = line.find("current parkingstatetype:") + len("current parkingstatetype:")
            parking_state = line[indx:].strip()
        if line.find("gear:") != -1:
            gear = line.split(" ")[-1].strip()
            if gear == "3":
                trajectory_x = []
                trajectory_y = []
            if gear.isdigit():
                vehicle_gear = string.atoi(gear)
        get_trajectory(line)
        process_envmod_msg(line)
    plt.ioff()
    plt.show()

def listdir(path, list_keyword):
    """
    return list_keyword file in path dir
    """
    list_file = []
    
    for file_name in os.listdir(path):
        file_path = os.path.join(path, file_name)
        if os.path.isdir(file_path):
            pass
        else :
            for keyword in list_keyword:
                if file_name.find(keyword) != -1:
                    list_file.append(file_path)
    return list_file

def merge_file(list_file):
    line_dict = {}
    base_tm = datetime(1970, 1, 1, 0, 0, 0)
    for file in list_file:
        fp_read = open(file, "r")
        for line in fp_read.readlines():
            for word in keywords:
                if line.find(word) != -1:
                    tm = get_time(line)
                    if tm >= base_tm:
                         line_dict[tm] = line
    return line_dict

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print ("Usage: python log_play.py [folder] [delta_sec]")
        delta_sec = -1
    elif len(sys.argv) == 2:
        delta_sec = 0
    else:
        delta_sec = string.atof(sys.argv[2])
    print ("folder:", sys.argv[1])
    print ("delta_sec:", delta_sec)
    list_file = listdir(sys.argv[1], ["plan", "ctrl"])
    line_dict = merge_file(list_file)
    process_dict(line_dict)