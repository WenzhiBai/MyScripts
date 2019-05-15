import numpy as np
import matplotlib.pyplot as plt
import math

ref_park_width_2 = 6.0 / 2.0
ref_park_depth = 2.2

car_wheel_base = 2.67
car_width_2 = 1.77 / 2.0
car_wheel_to_bottom = 1.04
car_wheel_to_top = 3.7

car_x_gt = 0.0
car_y_gt = 0.0
car_yaw_gt = 0.0

park_x_gt = 0.0
park_y_gt = 0.0
park_yaw_gt = 0.0

park_x_mea = 0.0
park_y_mea = 0.0
park_yaw_mea = 0.0

def quaternions_to_yaw(qw, qx, qy, qz):
    yaw = np.arctan2((2 * qx * qy + 2 * qw * qz), (1 - 2 * qy * qy - 2 * qz * qz))
    return yaw

def get_car_point(x, y, yaw):
    global car_width_2
    global car_wheel_to_bottom
    global car_wheel_to_top
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)

    car_x = []
    car_y = []
    car_x.append(-car_wheel_to_bottom * cos_yaw - car_width_2 * sin_yaw + x)
    car_x.append(-car_wheel_to_bottom * cos_yaw + car_width_2 * sin_yaw + x)
    car_x.append(car_wheel_to_top * cos_yaw + car_width_2 * sin_yaw + x)
    car_x.append(car_wheel_to_top * cos_yaw - car_width_2 * sin_yaw + x)
    car_x.append(-car_wheel_to_bottom * cos_yaw - car_width_2 * sin_yaw + x)

    car_y.append(-car_wheel_to_bottom * sin_yaw + car_width_2 * cos_yaw + y)
    car_y.append(-car_wheel_to_bottom * sin_yaw - car_width_2 * cos_yaw + y)
    car_y.append(car_wheel_to_top * sin_yaw - car_width_2 * cos_yaw + y)
    car_y.append(car_wheel_to_top * sin_yaw + car_width_2 * cos_yaw + y)
    car_y.append(-car_wheel_to_bottom * sin_yaw + car_width_2 * cos_yaw + y)

    return car_x, car_y

def get_park_point(x, y, yaw):
    global ref_park_width_2
    global ref_park_depth
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)

    park_x = []
    park_y = []
    park_x.append(-ref_park_width_2 * cos_yaw + x)
    park_x.append(-ref_park_width_2 * cos_yaw + ref_park_depth * sin_yaw + x)
    park_x.append(ref_park_width_2 * cos_yaw + ref_park_depth * sin_yaw + x)
    park_x.append(ref_park_width_2 * cos_yaw + x)
    park_x.append(-ref_park_width_2 * cos_yaw + x)

    park_y.append(-ref_park_width_2 * sin_yaw + y)
    park_y.append(-ref_park_width_2 * sin_yaw - ref_park_depth * cos_yaw + y)
    park_y.append(ref_park_width_2 * sin_yaw - ref_park_depth * cos_yaw + y)
    park_y.append(ref_park_width_2 * sin_yaw + y)
    park_y.append(-ref_park_width_2 * sin_yaw + y)

    return park_x, park_y

def draw_all():
    global car_x_gt
    global car_y_gt
    global car_yaw_gt
    global park_x_gt
    global park_y_gt
    global park_yaw_gt
    global park_x_mea
    global park_y_mea
    global park_yaw_mea
    car_x_gt_list, car_y_gt_list = get_car_point(car_x_gt, car_y_gt, car_yaw_gt)
    park_x_gt_list, park_y_gt_list = get_park_point(park_x_gt, park_y_gt, park_yaw_gt)
    park_x_mea_list, park_y_mea_list = get_park_point(park_x_mea, park_y_mea, park_yaw_mea)
    plt.plot(car_x_gt_list, car_y_gt_list, 'b-')
    plt.plot(park_x_gt_list, park_y_gt_list, 'r-')
    plt.plot(park_x_mea_list, park_y_mea_list, 'r--')
    plt.axis('equal')
    plt.show()


park_x_gt = ref_park_width_2
park_y_gt = 0.0
park_yaw_gt = 0.0

rear_wheel_x = float(input("请输入后车轮在车位坐标系下的测量值x:"))
rear_wheel_y = float(input("请输入后车轮在车位坐标系下的测量值y:"))
front_wheel_x = float(input("请输入前车轮在车位坐标系下的测量值x:"))
front_wheel_y = float(input("请输入前车轮在车位坐标系下的测量值y:"))
car_yaw_gt = np.arctan2(front_wheel_y - rear_wheel_y, front_wheel_x - rear_wheel_x)
car_x_gt = rear_wheel_x + car_width_2 * np.cos(car_yaw_gt + math.pi / 2)
car_y_gt = rear_wheel_y + car_width_2 * np.sin(car_yaw_gt + math.pi / 2)

car_x_mea_w = float(input("请输入车辆位姿在世界坐标系下的记录值x:"))
car_y_mea_w = float(input("请输入车辆位姿在世界坐标系下的记录值y:"))
car_qx_mea_w = float(input("请输入车辆位姿在世界坐标系下的记录值qx:"))
car_qy_mea_w = float(input("请输入车辆位姿在世界坐标系下的记录值qy:"))
car_qz_mea_w = float(input("请输入车辆位姿在世界坐标系下的记录值qz:"))
car_qw_mea_w = float(input("请输入车辆位姿在世界坐标系下的记录值qw:"))
car_yaw_mea_w = quaternions_to_yaw(car_qw_mea_w, car_qx_mea_w, car_qy_mea_w, car_qz_mea_w)

point_c_x_w = float(input("请输入车位C点在世界坐标系下的记录值x:"))
point_c_y_w = float(input("请输入车位C点在世界坐标系下的记录值y:"))
point_d_x_w = float(input("请输入车位D点在世界坐标系下的记录值x:"))
point_d_y_w = float(input("请输入车位D点在世界坐标系下的记录值y:"))

park_x_mea_w = (point_c_x_w + point_d_x_w) / 2.0
park_y_mea_w = (point_c_y_w + point_d_y_w) / 2.0
park_theta_mea_w = np.arctan2(point_c_y_w - point_d_y_w, point_c_x_w - point_d_x_w)

theta_w_c = car_yaw_mea_w
cos_theta_w_c = np.cos(theta_w_c)
sin_theta_w_c = np.sin(theta_w_c)
park_theta_mea_c = park_theta_mea_w - theta_w_c
park_x_mea_c = cos_theta_w_c * (park_x_mea_w - car_x_mea_w) + sin_theta_w_c * (park_y_mea_w - car_y_mea_w)
park_y_mea_c = -sin_theta_w_c * (park_x_mea_w - car_x_mea_w) + cos_theta_w_c * (park_y_mea_w - car_y_mea_w)

theta_p_c = car_yaw_gt - park_yaw_gt
cos_theta_p_c = np.cos(theta_p_c)
sin_theta_p_c = np.sin(theta_p_c)
park_yaw_mea = park_theta_mea_c - theta_p_c
park_x_mea = cos_theta_p_c * park_x_mea_c - sin_theta_p_c * park_y_mea_c + car_x_gt
park_y_mea = sin_theta_p_c * park_x_mea_c + cos_theta_p_c * park_y_mea_c + car_y_gt

mea_error = 1 - np.sqrt((front_wheel_y - rear_wheel_y)**2 + (front_wheel_x - rear_wheel_x)**2) / car_wheel_base
print("测量误差:", mea_error)
print("车辆x方向误差:", park_x_mea - park_x_gt)
print("车辆y方向误差:", park_y_mea - park_y_gt)
print("车辆yaw方向误差:", math.degrees(park_yaw_mea - park_yaw_gt))
draw_all()
