import numpy as np
import matplotlib.pyplot as plt
import math

car_wheel_base = 2.67
car_width_2 = 1.77 / 2.0
car_wheel_to_bottom = 1.04
car_wheel_to_top = 3.7

car_x_gt = 0.0
car_y_gt = 0.0
car_yaw_gt = 0.0

car_x_mea = 0.0
car_y_mea = 0.0
car_yaw_mea = 0.0

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

def draw_all():
    global car_x_gt
    global car_y_gt
    global car_yaw_gt
    global car_x_mea
    global car_y_mea
    global car_yaw_mea
    x_gt, y_gt = get_car_point(car_x_gt, car_y_gt, car_yaw_gt)
    x_mea, y_mea = get_car_point(car_x_mea, car_y_mea, car_yaw_mea)
    plt.plot(x_gt, y_gt, 'r')
    plt.plot(x_mea, y_mea, 'b')
    plt.axis('equal')
    plt.show()


rear_wheel_x = float(input("请输入右后车轮在车位坐标系下的测量值x:"))
rear_wheel_y = float(input("请输入右后车轮在车位坐标系下的测量值y:"))
front_wheel_x = float(input("请输入右前车轮在车位坐标系下的测量值x:"))
front_wheel_y = float(input("请输入右前车轮在车位坐标系下的测量值y:"))
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
x_w_p = point_d_x_w
y_w_p = point_d_y_w
theta_w_p = np.arctan2(point_c_y_w - point_d_y_w, point_c_x_w - point_d_x_w)
cos_theta_w_p = np.cos(theta_w_p)
sin_theta_w_p = np.sin(theta_w_p)

car_yaw_mea = car_yaw_mea_w - theta_w_p
car_x_mea = cos_theta_w_p * (car_x_mea_w - x_w_p) + sin_theta_w_p * (car_y_mea_w - y_w_p)
car_y_mea = -sin_theta_w_p * (car_x_mea_w - x_w_p) + cos_theta_w_p * (car_y_mea_w - y_w_p)

mea_error = np.sqrt((front_wheel_y - rear_wheel_y)**2 + (front_wheel_x - rear_wheel_x)**2) / car_wheel_base
print("测量误差:", mea_error)
print("车辆x方向误差:", car_x_mea - car_x_gt)
print("车辆y方向误差:", car_y_mea - car_y_gt)
print("车辆yaw方向误差:", math.degrees(car_yaw_mea - car_yaw_gt))
draw_all()
