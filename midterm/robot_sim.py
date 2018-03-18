#!/usr/bin/env python3

from PyQt5.QtWidgets import QWidget, QPushButton, QApplication, QLabel, QLineEdit, QMessageBox
# from PyQt5.QtGui import QColor
import math
import pygame
import random
import sys

pygame.init()
fps = pygame.time.Clock()


WHITE = (255, 255, 255)
ORANGE = (255,140,0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
BLACK = (0, 0, 0)

FRAMERATE = 60
PIX_PER_FOOT = 40
CAR_LENGTH = 4
CAR_WIDTH = 2
CAR_HALF_LENGTH = 2
CAR_HALF_WIDTH = 1
WHEEL_RADIUS = 0.5
MAX_SPEED = 15 # ft/s
MAX_ACCELERATION = 50 # ft/s/s
DIST_PRECISION = 0.01 # ft
YAW_PRECISION = 0.1 # deg
SPEED_PRECISION = 0.0001 # ft/s
SCREEN_WIDTH = 30 * PIX_PER_FOOT
SCREEN_HEIGHT = 15 * PIX_PER_FOOT
SCREEN_BUFFER = 2 * PIX_PER_FOOT
ACTUAL_PATH_LIMIT = 36000 # limit on history kept for path travelled
robot_pos = [0, 0]
ball_vel = [0, 0]
l_score = 0
r_score = 0
control_mode = 'none'
screen_center = (0, 0)

def str_to_float(string):
    if len(string) == 0:
        return 0.0
    else:
        return float(string)

class Car():
    def __init__(self, xpos, ypos, yaw=0):
        # SCREEN_WIDTH = 1ft
        # length = 2ft
        self.xpos = xpos
        self.ypos = ypos
        self.xpos_actual = 0
        self.ypos_actual = 0
        self.yaw = yaw
        self.psi_1 = 0 # rad/s
        self.psi_2 = 0 # rad/s
        self.psi_3 = 0 # rad/s
        self.psi_4 = 0 # rad/s
        self.x_vel = 0 # ft/s
        self.y_vel = 0 # ft/s
        self.r_vel = 0 # deg/s
        self.integral = 0
        self.desired_path = []
        self.actual_path = []
        # self.surface = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT))

    def corners(self):
        yaw_rad = math.radians(self.yaw)
        cos_yaw = math.cos(yaw_rad)
        sin_yaw = math.sin(yaw_rad)
        p1_x = self.xpos + (-CAR_HALF_WIDTH * cos_yaw + CAR_HALF_LENGTH * sin_yaw) * PIX_PER_FOOT
        p2_x = self.xpos + (CAR_HALF_WIDTH * cos_yaw + CAR_HALF_LENGTH * sin_yaw) * PIX_PER_FOOT
        p3_x = self.xpos + (-CAR_HALF_WIDTH * cos_yaw - CAR_HALF_LENGTH * sin_yaw) * PIX_PER_FOOT
        p4_x = self.xpos + (CAR_HALF_WIDTH * cos_yaw - CAR_HALF_LENGTH * sin_yaw) * PIX_PER_FOOT
        p1_y = self.ypos + (CAR_HALF_LENGTH * cos_yaw + CAR_HALF_WIDTH * sin_yaw) * PIX_PER_FOOT
        p2_y = self.ypos + (CAR_HALF_LENGTH * cos_yaw - CAR_HALF_WIDTH * sin_yaw) * PIX_PER_FOOT
        p3_y = self.ypos + (-CAR_HALF_LENGTH * cos_yaw + CAR_HALF_WIDTH * sin_yaw) * PIX_PER_FOOT
        p4_y = self.ypos + (-CAR_HALF_LENGTH * cos_yaw - CAR_HALF_WIDTH * sin_yaw) * PIX_PER_FOOT
        return [[p2_x, p2_y], [p1_x, p1_y], [p3_x, p3_y], [p4_x, p4_y]]

    def front_point(self):
        yaw_rad = math.radians(self.yaw)
        p_y = int(self.ypos - (1.8 * math.cos(yaw_rad)) * PIX_PER_FOOT)
        p_x = int(self.xpos - (1.8 * math.sin(yaw_rad)) * PIX_PER_FOOT)
        return [p_x, p_y]

    def position(self):
        return (self.xpos, self.ypos)

    def set_position(self, xpos, ypos):
        self.xpos = xpos
        self.ypos = ypos

    def reset_screen_pos(self):
        global screen_center
        screen_center = (self.xpos_actual, self.ypos_actual)
        # screen_center[0] = self.xpos_actual
        # screen_center[1] = self.ypos_actual
        self.set_position(SCREEN_WIDTH/2, SCREEN_HEIGHT/2)

    def update(self):
        global control_mode
        self.xpos += self.x_vel * PIX_PER_FOOT * (1 / FRAMERATE)
        self.ypos -= self.y_vel * PIX_PER_FOOT * (1 / FRAMERATE)
        if self.xpos < SCREEN_BUFFER or self.xpos > SCREEN_WIDTH - SCREEN_BUFFER:
            self.reset_screen_pos()
        if self.ypos < SCREEN_BUFFER or self.ypos > SCREEN_HEIGHT - SCREEN_BUFFER:
            self.reset_screen_pos()
        self.xpos_actual += self.x_vel * (1 / FRAMERATE)
        self.ypos_actual += self.y_vel * (1 / FRAMERATE)
        self.actual_path.append((self.xpos_actual, self.ypos_actual))
        if len(self.actual_path) > ACTUAL_PATH_LIMIT:
            del self.actual_path[0]
        self.yaw += self.r_vel * (1 / FRAMERATE)
        if self.yaw > 360 or self.yaw < 0:
            self.yaw %= 360
        if control_mode == 'manual_vel':
            self.find_psi_from_desired_vel_cartesian(self.x_vel, self.y_vel, self.r_vel)
        if control_mode == 'manual_psi':
            self.find_vel_from_psi()
        if control_mode == 'point_execution':
            dist_x = self.dest_x - self.xpos_actual
            dist_y = self.dest_y - self.ypos_actual
            dist_yaw = self.dest_orientation - self.yaw
            dist_yaw_abs = min(math.fabs(dist_yaw), math.fabs(dist_yaw % 360))
            dist = math.sqrt(dist_x * dist_x + dist_y * dist_y)
            speed = math.sqrt(self.x_vel * self.x_vel + self.y_vel * self.y_vel)
            if speed * speed / (2 * MAX_ACCELERATION) > dist:
                control_mode = 'slowdown'
                print('slowing down')
                self.accelerate(0, 0)
                self.r_vel = 0
                self.time_to_take = 0
                self.find_psi_from_desired_vel_cartesian(self.x_vel, self.y_vel, self.r_vel)
                return
            if math.fabs(dist_x) <= DIST_PRECISION and math.fabs(dist_y) <= DIST_PRECISION and dist_yaw_abs <= YAW_PRECISION:
                control_mode = 'slowdown'
                print('slowing down')
                self.accelerate(0, 0)
                self.r_vel = 0
                self.time_to_take = 0
                self.find_psi_from_desired_vel_cartesian(self.x_vel, self.y_vel, self.r_vel)
                return
            self.accelerate(dist_x / self.time_to_take, dist_y / self.time_to_take)
            dist_yaw %= 360
            if dist_yaw < 180:
                self.r_vel = dist_yaw / self.time_to_take
            else:
                self.r_vel = -1 * dist_yaw / self.time_to_take
            self.find_psi_from_desired_vel_cartesian(self.x_vel, self.y_vel, self.r_vel)
            self.time_to_take -= (1 / FRAMERATE)
            if self.time_to_take < 0:
                self.time_to_take = 1
        if control_mode == 'circle_execution':
            if not self.direction_set:
                dist_yaw = self.circle_center_dir - self.yaw
                dist_yaw_abs = min(math.fabs(dist_yaw), math.fabs(dist_yaw % 360))
                if dist_yaw_abs <= YAW_PRECISION:
                    self.direction_set = True
                    return
                dist_yaw %= 360
                if dist_yaw > 180:
                    self.psi_1 = 1
                    self.psi_2 = -1
                    self.psi_3 = 1
                    self.psi_4 = -1
                else:
                    self.psi_1 = -1
                    self.psi_2 = 1
                    self.psi_3 = -1
                    self.psi_4 = 1
                self.find_vel_from_psi()
                return
            angular_vel = 2 * math.pi / self.time_to_take
            linear_vel = angular_vel * self.desired_circle_radius
            inverse_rad = 1 / WHEEL_RADIUS
            size_const = (CAR_HALF_LENGTH + CAR_HALF_WIDTH)
            self.psi_1 = inverse_rad * (linear_vel + size_const * angular_vel)
            self.psi_2 = inverse_rad * (linear_vel - size_const * angular_vel)
            self.psi_3 = inverse_rad * (linear_vel + size_const * angular_vel)
            self.psi_4 = inverse_rad * (linear_vel - size_const * angular_vel)
            self.find_vel_from_psi()
            return
        if control_mode == 'rectangle_execution':
            dest = self.rect_points[(self.last_rect_point + 1) % 4]
            dist_x = dest[0] - self.xpos_actual
            dist_y = dest[1] - self.ypos_actual
            dist = math.sqrt(dist_x * dist_x + dist_y * dist_y)
            speed = math.sqrt(self.x_vel * self.x_vel + self.y_vel * self.y_vel)
            self.r_vel = 0
            if speed * speed / (2 * MAX_ACCELERATION) > dist:
                # self.last_rect_point += 1
                print('slowing down')
                self.accelerate(0, 0)
                self.find_psi_from_desired_vel_cartesian(self.x_vel, self.y_vel, self.r_vel)
                if math.fabs(dist_x) <= 10 * DIST_PRECISION and math.fabs(dist_y) <= 10 * DIST_PRECISION:
                    self.last_rect_point += 1
                    return
            else:
                total_rect_dist = 2 * (self.rect_side1 + self.rect_side2)
                avg_rect_vel = total_rect_dist / self.time_to_take
                self.accelerate(avg_rect_vel * dist_x / dist, avg_rect_vel * dist_y / dist)
                self.find_psi_from_desired_vel_cartesian(self.x_vel, self.y_vel, self.r_vel)
            # self.time_to_take -= (1 / FRAMERATE)
            # if self.time_to_take < 0:
            #     self.time_to_take = 1
            return
        if control_mode == 'slowdown':
            self.accelerate(0, 0)
            self.r_vel = 0
            self.time_to_take = 0
            self.find_psi_from_desired_vel_cartesian(self.x_vel, self.y_vel, self.r_vel)
            if math.fabs(self.x_vel) <= SPEED_PRECISION and math.fabs(self.y_vel) <= SPEED_PRECISION:
                control_mode = 'none'
                self.x_vel = 0
                self.y_vel = 0
            dist_x = self.dest_x - self.xpos_actual
            dist_y = self.dest_y - self.ypos_actual
            dist_yaw = self.dest_orientation - self.yaw
            if math.fabs(dist_x) > DIST_PRECISION or math.fabs(dist_y) > DIST_PRECISION:
                control_mode = 'point_execution'
                self.time_to_take = 1



    def find_vel_from_psi(self):
        rframe_x_vel = WHEEL_RADIUS * (self.psi_1 - self.psi_2 - self.psi_3 + self.psi_4) / 4
        rframe_y_vel = WHEEL_RADIUS * (self.psi_1 + self.psi_2 + self.psi_3 + self.psi_4) / 4
        yaw_rad = math.radians(self.yaw)
        cos_yaw = math.cos(yaw_rad)
        sin_yaw = math.sin(yaw_rad)
        self.x_vel = cos_yaw * rframe_x_vel - sin_yaw * rframe_y_vel
        self.y_vel = cos_yaw * rframe_y_vel + sin_yaw * rframe_x_vel
        self.r_vel = (180 / math.pi) * WHEEL_RADIUS * (self.psi_2 - self.psi_3 + self.psi_4 - self.psi_1) / \
                     (4 * (CAR_HALF_LENGTH + CAR_HALF_WIDTH))
        return

    def find_psi_from_vel(self):
        yaw_rad = math.radians(self.yaw)
        return

    def find_psi_from_desired_vel_cartesian(self, x_vel, y_vel, r_vel):
        speed = math.sqrt(x_vel * x_vel + y_vel * y_vel)
        direction = math.degrees(math.atan2(y_vel, x_vel))
        if direction < 0:
            direction += 360
        self.find_psi_from_desired_vel_polar(speed, direction)
        self.r_vel = r_vel
        return

    def find_psi_from_desired_vel_polar(self, speed, direction):
        yaw_rad = math.radians(self.yaw)
        dir_rad = math.radians(direction)
        rvel_rad = math.radians(self.r_vel)
        V_cx = speed * math.cos(dir_rad - (yaw_rad + rvel_rad * (1 / FRAMERATE)))
        V_cy = speed * math.sin(dir_rad - (yaw_rad + rvel_rad * (1 / FRAMERATE)))
        omega_c = rvel_rad
        self.psi_1 = (1 / WHEEL_RADIUS) * (V_cy + V_cx - (CAR_HALF_LENGTH + CAR_HALF_WIDTH) * omega_c)
        self.psi_2 = (1 / WHEEL_RADIUS) * (V_cy - V_cx + (CAR_HALF_LENGTH + CAR_HALF_WIDTH) * omega_c)
        self.psi_3 = (1 / WHEEL_RADIUS) * (V_cy - V_cx - (CAR_HALF_LENGTH + CAR_HALF_WIDTH) * omega_c)
        self.psi_4 = (1 / WHEEL_RADIUS) * (V_cy + V_cx + (CAR_HALF_LENGTH + CAR_HALF_WIDTH) * omega_c)
        return

    def stop(self):
        self.psi_1 = 0
        self.psi_2 = 0
        self.psi_3 = 0
        self.psi_4 = 0
        self.x_vel = 0
        self.y_vel = 0
        self.r_vel = 0

    def accelerate(self, desired_x_vel, desired_y_vel):
        desired_speed = math.sqrt(desired_x_vel ** 2 + desired_y_vel ** 2)
        if desired_speed > MAX_SPEED:
            print('Warning: Desired speed is greater than 15 ft/s')
            desired_x_vel = desired_x_vel * MAX_SPEED / desired_speed
            desired_y_vel = desired_y_vel * MAX_SPEED / desired_speed
        x_vel_delta = desired_x_vel - self.x_vel
        y_vel_delta = desired_y_vel - self.y_vel
        speed_delta = math.sqrt(x_vel_delta * x_vel_delta + y_vel_delta * y_vel_delta)
        speed_delta = min(speed_delta, MAX_ACCELERATION / FRAMERATE)
        accel_direction = math.atan2(y_vel_delta, x_vel_delta)
        x_accel = speed_delta * math.cos(accel_direction)
        y_accel = speed_delta * math.sin(accel_direction)
        self.x_vel += x_accel
        self.y_vel += y_accel
        return

    def direction_to_point(self, point_x, point_y):
        y_diff = point_y - self.ypos
        x_diff = point_x - self.xpos
        direction = math.degrees(math.atan2(y_diff, x_diff))
        return direction

    def pid_control(self):
        distance_x = self.dest_x - self.xpos_actual
        distance_y = self.dest_y - self.ypos_actual
        distance_yaw = self.dest_orientation - self.yaw
        x_vel = distance_x / self.time_to_take
        y_vel = distance_y / self.time_to_take
        r_vel = distance_yaw / self.time_to_take

class RobotMenu(QWidget):
    
    def __init__(self):
        super().__init__()
        
        self.initUI()

    def closeEvent(self, event):
        exit(0)
        event.accept()
        
        
    def initUI(self):

        # Specify Psi Values
        self.psi_1_lbl = QLabel(self)
        self.psi_1_lbl.setText('Desired Psi 1 (rad/s): ')
        self.psi_1_field = QLineEdit(self)
        self.psi_1_lbl.move(10, 40)
        self.psi_1_field.move(120, 36)

        self.psi_2_lbl = QLabel(self)
        self.psi_2_lbl.setText('Desired Psi 2 (rad/s): ')
        self.psi_2_field = QLineEdit(self)
        self.psi_2_lbl.move(10, 60)
        self.psi_2_field.move(120, 56)

        self.psi_3_lbl = QLabel(self)
        self.psi_3_lbl.setText('Desired Psi 3 (rad/s): ')
        self.psi_3_field = QLineEdit(self)
        self.psi_3_lbl.move(10, 80)
        self.psi_3_field.move(120, 76)

        self.psi_4_lbl = QLabel(self)
        self.psi_4_lbl.setText('Desired Psi 4 (rad/s): ')
        self.psi_4_field = QLineEdit(self)
        self.psi_4_lbl.move(10, 100)
        self.psi_4_field.move(120, 96)
        
        self.set_psi_button = QPushButton('Set Psi', self)
        self.set_psi_button.move(120, 120)
        self.set_psi_button.clicked[bool].connect(self.setPsi)

        # Specify X and Y Velocities
        self.x_vel_lbl = QLabel(self)
        self.x_vel_lbl.setText('Desired X Vel (ft/s): ')
        self.x_vel_field = QLineEdit(self)
        self.x_vel_lbl.move(10, 160)
        self.x_vel_field.move(150, 156)

        self.y_vel_lbl = QLabel(self)
        self.y_vel_lbl.setText('Desired Y Vel (ft/s): ')
        self.y_vel_field = QLineEdit(self)
        self.y_vel_lbl.move(10, 180)
        self.y_vel_field.move(150, 176)

        self.r_vel_lbl = QLabel(self)
        self.r_vel_lbl.setText('Desired Angular Vel (deg/s): ')
        self.r_vel_field = QLineEdit(self)
        self.r_vel_lbl.move(10, 200)
        self.r_vel_field.move(150, 196)
        
        self.set_vel_button = QPushButton('Set Vel', self)
        self.set_vel_button.move(150, 220)
        self.set_vel_button.clicked[bool].connect(self.setVel)

        # Specify Velocity and Direction
        self.speed_lbl = QLabel(self)
        self.speed_lbl.setText('Desired Speed (ft/s): ')
        self.speed_field = QLineEdit(self)
        self.speed_lbl.move(10, 280)
        self.speed_field.move(150, 276)

        self.direction_lbl = QLabel(self)
        self.direction_lbl.setText('Desired Angle (degrees): ')
        self.direction_field = QLineEdit(self)
        self.direction_lbl.move(10, 300)
        self.direction_field.move(150, 296)

        self.r_polar_vel_lbl = QLabel(self)
        self.r_polar_vel_lbl.setText('Desired Angular Vel (deg/s): ')
        self.r_polar_vel_field = QLineEdit(self)
        self.r_polar_vel_lbl.move(10, 320)
        self.r_polar_vel_field.move(150, 316)
        
        self.set_polar_vel_button = QPushButton('Set Vel', self)
        self.set_polar_vel_button.move(150, 340)
        self.set_polar_vel_button.clicked[bool].connect(self.setPolarVel)

        # Specify Point to Move to
        self.dest_point_x_lbl = QLabel(self)
        self.dest_point_x_lbl.setText('Desired X Coord (ft): ')
        self.dest_point_x_field = QLineEdit(self)
        self.dest_point_x_lbl.move(10, 380)
        self.dest_point_x_field.move(160, 376)

        self.dest_point_y_lbl = QLabel(self)
        self.dest_point_y_lbl.setText('Desired Y Coord (ft): ')
        self.dest_point_y_field = QLineEdit(self)
        self.dest_point_y_lbl.move(10, 400)
        self.dest_point_y_field.move(160, 396)

        self.dest_orientation_lbl = QLabel(self)
        self.dest_orientation_lbl.setText('Desired End Orientation (deg): ')
        self.dest_orientation_field = QLineEdit(self)
        self.dest_orientation_lbl.move(10, 420)
        self.dest_orientation_field.move(160, 416)

        self.time_to_take_lbl = QLabel(self)
        self.time_to_take_lbl.setText('Desired Amount of Time (s): ')
        self.time_to_take_field = QLineEdit(self)
        self.time_to_take_lbl.move(10, 440)
        self.time_to_take_field.move(160, 436)
        
        self.set_dest_button = QPushButton('Set Path', self)
        self.set_dest_button.move(160, 460)
        self.set_dest_button.clicked[bool].connect(self.setDest)

        # Specify Circle to Move in
        self.circle_radius_lbl = QLabel(self)
        self.circle_radius_lbl.setText('Desired Radius (ft): ')
        self.circle_radius_field = QLineEdit(self)
        self.circle_radius_lbl.move(10, 500)
        self.circle_radius_field.move(160, 496)

        self.circle_center_dir_lbl = QLabel(self)
        self.circle_center_dir_lbl.setText('Direction of circle center (deg): ')
        self.circle_center_dir_field = QLineEdit(self)
        self.circle_center_dir_lbl.move(10, 520)
        self.circle_center_dir_field.move(160, 516)

        self.circle_time_lbl = QLabel(self)
        self.circle_time_lbl.setText('Time per Revolution (s): ')
        self.circle_time_field = QLineEdit(self)
        self.circle_time_lbl.move(10, 540)
        self.circle_time_field.move(160, 536)
        
        self.set_circle_button = QPushButton('Set Path', self)
        self.set_circle_button.move(160, 560)
        self.set_circle_button.clicked[bool].connect(self.setCircle)

        # Specify Rectangle to Move in
        self.rectangle_side1_lbl = QLabel(self)
        self.rectangle_side1_lbl.setText('First Side Length (ft): ')
        self.rectangle_side1_field = QLineEdit(self)
        self.rectangle_side1_lbl.move(10, 600)
        self.rectangle_side1_field.move(160, 596)

        self.rectangle_side2_lbl = QLabel(self)
        self.rectangle_side2_lbl.setText('Second Side Length (ft): ')
        self.rectangle_side2_field = QLineEdit(self)
        self.rectangle_side2_lbl.move(10, 620)
        self.rectangle_side2_field.move(160, 616)

        self.rectangle_inclination_lbl = QLabel(self)
        self.rectangle_inclination_lbl.setText('Inclination (deg): ')
        self.rectangle_inclination_field = QLineEdit(self)
        self.rectangle_inclination_lbl.move(10, 640)
        self.rectangle_inclination_field.move(160, 636)

        self.rectangle_time_lbl = QLabel(self)
        self.rectangle_time_lbl.setText('Time per Revolution (s): ')
        self.rectangle_time_field = QLineEdit(self)
        self.rectangle_time_lbl.move(10, 660)
        self.rectangle_time_field.move(160, 656)
        
        self.set_rectangle_button = QPushButton('Set Path', self)
        self.set_rectangle_button.move(160, 680)
        self.set_rectangle_button.clicked[bool].connect(self.setRectangle)

        # Specify Rectangle to Travel On

        self.setGeometry(200, 200, 500, 800)
        self.setWindowTitle('Robot Menu')
        self.show()

    def setPsi(self, pressed):
        global my_car, control_mode
        my_car.psi_1 = str_to_float(self.psi_1_field.text())
        my_car.psi_2 = str_to_float(self.psi_2_field.text())
        my_car.psi_3 = str_to_float(self.psi_3_field.text())
        my_car.psi_4 = str_to_float(self.psi_4_field.text())
        my_car.find_vel_from_psi()
        control_mode = 'manual_psi'

    def setVel(self, pressed):
        global my_car, control_mode
        my_car.x_vel = str_to_float(self.x_vel_field.text())
        my_car.y_vel = str_to_float(self.y_vel_field.text())
        my_car.r_vel = str_to_float(self.r_vel_field.text())
        my_car.find_psi_from_desired_vel_cartesian(my_car.x_vel, my_car.y_vel, my_car.r_vel)
        my_car.find_vel_from_psi()
        control_mode = 'manual_vel'

    def setPolarVel(self, pressed):
        global my_car, control_mode
        speed = str_to_float(self.speed_field.text())
        direction = str_to_float(self.direction_field.text())
        my_car.r_vel = str_to_float(self.r_vel_field.text())
        my_car.find_psi_from_desired_vel_polar(speed, direction)
        my_car.find_vel_from_psi()
        control_mode = 'manual_vel'

    def setDest(self, time):
        global my_car, control_mode
        dest_x = str_to_float(self.dest_point_x_field.text())
        dest_y = str_to_float(self.dest_point_y_field.text())
        dest_orientation = str_to_float(self.dest_orientation_field.text())
        dest_orientation %= 360
        time_to_take = str_to_float(self.time_to_take_field.text())
        if time_to_take <= 0:
            self.alert_msg = QMessageBox()
            self.alert_msg.setIcon(QMessageBox.Information)
            self.alert_msg.setWindowTitle("Simulation Error")
            self.alert_msg.setText('An error has occurred')
            self.alert_msg.setInformativeText('Zero or negative time specified: %fs' % time_to_take)
            self.alert_msg.setStandardButtons(QMessageBox.Ok)
            # self.alert_msg.buttonClicked.connect(msgbtn)
            retval = self.alert_msg.exec_()
            return

        # check for speed > 15 ft/s
        dist_x = dest_x - my_car.xpos_actual
        dist_y = dest_y - my_car.ypos_actual
        distance = math.sqrt(dist_x * dist_x + dist_y * dist_y)
        if (distance / time_to_take) > MAX_SPEED:
            # alert('Desired execution requires a speed greater than 15 ft/s')
            self.alert_msg = QMessageBox()
            self.alert_msg.setIcon(QMessageBox.Information)
            self.alert_msg.setWindowTitle("Simulation Error")
            self.alert_msg.setText('An error has occurred')
            self.alert_msg.setInformativeText('Desired execution requires a speed greater than %f ft/s' % MAX_SPEED)
            self.alert_msg.setStandardButtons(QMessageBox.Ok)
            # self.alert_msg.buttonClicked.connect(msgbtn)
            retval = self.alert_msg.exec_()
            return
        my_car.dest_x = dest_x
        my_car.dest_y = dest_y
        my_car.dest_orientation = dest_orientation
        my_car.time_to_take = time_to_take
        control_mode = 'point_execution'
        return

    def setCircle(self):
        global my_car, control_mode
        circle_radius = str_to_float(self.circle_radius_field.text())
        circle_center_dir = str_to_float(self.circle_center_dir_field.text())
        circle_center_dir %= 360
        # circle_dest_orientation = str_to_float(self.circle_dest_orientation_field.text())
        circle_time = str_to_float(self.circle_time_field.text())
        if circle_time <= 0:
            self.alert_msg = QMessageBox()
            self.alert_msg.setIcon(QMessageBox.Information)
            self.alert_msg.setWindowTitle("Simulation Error")
            self.alert_msg.setText('An error has occurred')
            self.alert_msg.setInformativeText('Zero or negative time specified: %fs' % time_to_take)
            self.alert_msg.setStandardButtons(QMessageBox.Ok)
            retval = self.alert_msg.exec_()
            return
        angular_vel = 2 * math.pi / circle_time
        linear_vel = angular_vel * circle_radius
        if linear_vel > MAX_SPEED:
            # alert('Desired execution requires a speed greater than 15 ft/s')
            self.alert_msg = QMessageBox()
            self.alert_msg.setIcon(QMessageBox.Information)
            self.alert_msg.setWindowTitle("Simulation Error")
            self.alert_msg.setText('An error has occurred')
            self.alert_msg.setInformativeText('Desired execution requires a speed greater than %f ft/s' % MAX_SPEED)
            self.alert_msg.setStandardButtons(QMessageBox.Ok)
            # self.alert_msg.buttonClicked.connect(msgbtn)
            retval = self.alert_msg.exec_()
            return

        # circle_center_x = my_car.xpos_actual + circle_radius * math.cos(circle_center_dir)
        # circle_center_y = my_car.ypos_actual + circle_radius * math.sin(circle_center_dir)
        my_car.circle_center_dir = circle_center_dir
        # my_car.desired_circle_center = (circle_center_x, circle_center_y)
        my_car.desired_circle_radius = circle_radius
        my_car.time_to_take = circle_time
        my_car.direction_set = False
        control_mode = 'circle_execution'
        pass

    def setRectangle(self):
        global my_car, control_mode

        self.rectangle_side1_lbl
        self.rectangle_side2_lbl
        self.rectangle_inclination_lbl
        self.rectangle_time_lbl
        side1 = str_to_float(self.rectangle_side1_field.text())
        side2 = str_to_float(self.rectangle_side2_field.text())
        inclination = str_to_float(self.rectangle_inclination_field.text())
        rect_time = str_to_float(self.rectangle_time_field.text())
        if side1 <= 0 or side2 <= 0:
            self.alert_msg = QMessageBox()
            self.alert_msg.setIcon(QMessageBox.Information)
            self.alert_msg.setWindowTitle("Simulation Error")
            self.alert_msg.setText('An error has occurred')
            self.alert_msg.setInformativeText('Zero or negative side length specified: %f, %f' % (side1, side2))
            self.alert_msg.setStandardButtons(QMessageBox.Ok)
            retval = self.alert_msg.exec_()
            return
        if rect_time <= 0:
            self.alert_msg = QMessageBox()
            self.alert_msg.setIcon(QMessageBox.Information)
            self.alert_msg.setWindowTitle("Simulation Error")
            self.alert_msg.setText('An error has occurred')
            self.alert_msg.setInformativeText('Zero or negative time specified: %fs' % rect_time)
            self.alert_msg.setStandardButtons(QMessageBox.Ok)
            retval = self.alert_msg.exec_()
            return
        if 2 * (side1 + side2) / rect_time > MAX_SPEED:
            # alert('Desired execution requires a speed greater than 15 ft/s')
            self.alert_msg = QMessageBox()
            self.alert_msg.setIcon(QMessageBox.Information)
            self.alert_msg.setWindowTitle("Simulation Error")
            self.alert_msg.setText('An error has occurred')
            self.alert_msg.setInformativeText('Desired execution requires a speed greater than %f ft/s' % MAX_SPEED)
            self.alert_msg.setStandardButtons(QMessageBox.Ok)
            # self.alert_msg.buttonClicked.connect(msgbtn)
            retval = self.alert_msg.exec_()
            return
        angle_rad = math.radians(inclination)
        diag_len = math.sqrt(side1 ** 2 + side2 ** 2)
        phi = math.acos(side1 / diag_len) + angle_rad
        iphi = phi - math.pi / 2
        print(phi)
        print(iphi)
        point0 = (my_car.xpos_actual, my_car.ypos_actual)
        point1 = (my_car.xpos_actual + side1 * math.cos(phi), my_car.ypos_actual + side1 * math.sin(phi))
        point2 = (my_car.xpos_actual + diag_len * math.cos(angle_rad), my_car.ypos_actual + diag_len * math.sin(angle_rad))
        point3 = (my_car.xpos_actual + side2 * math.cos(iphi), my_car.ypos_actual + side2 * math.sin(iphi))
        my_car.rect_points = [point0, point1, point2, point3]
        print(my_car.rect_points)
        my_car.time_to_take = rect_time
        my_car.last_rect_point = 0
        my_car.rect_side1 = side1
        my_car.rect_side2 = side2
        control_mode = 'rectangle_execution'
        pass

    def setFigure8(self):
        global my_car, control_mode
        control_mode = 'figure8_execution'
        pass


my_car = Car(SCREEN_WIDTH/2, SCREEN_HEIGHT/2)

window = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), 0, 32)
pygame.display.set_caption('Robotics Simulation')

def robot_init(right):
    global robot_pos
    robot_pos = [SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2]

def init():
    if random.randrange(0, 2) == 0:
        robot_init(True)
    else:
        robot_init(False)

def reset():
    global my_car, control_mode, screen_center
    screen_center = (0, 0)
    my_car.set_position(SCREEN_WIDTH/2, SCREEN_HEIGHT/2)
    my_car.xpos_actual = 0
    my_car.ypos_actual = 0
    my_car.yaw = 0
    my_car.psi_1 = 0
    my_car.psi_2 = 0
    my_car.psi_3 = 0
    my_car.psi_4 = 0
    my_car.x_vel = 0
    my_car.y_vel = 0
    my_car.r_vel = 0
    my_car.actual_path = []
    control_mode = 'none'

def draw_point(canvas, color, pos):
    canvas.fill(color, (pos, (2, 2)))

def coord_global_to_frame(point):
    global screen_center
    screen_x = SCREEN_WIDTH/2 + PIX_PER_FOOT * (point[0] - screen_center[0])
    screen_y = SCREEN_HEIGHT/2 - PIX_PER_FOOT * (point[1] - screen_center[1])
    return (screen_x, screen_y)

def draw(canvas):
    global robot_pos, ball_vel, l_score, r_score, my_car, control_mode

    canvas.fill(BLACK)

    # 6X6 Grid
    margin = int(0.5 * PIX_PER_FOOT)
    for column in range(0, SCREEN_WIDTH, margin):
        for row in range(0, SCREEN_HEIGHT, margin):
            pygame.draw.rect(canvas, WHITE, [column,row,SCREEN_WIDTH,SCREEN_HEIGHT], 1)

    for point in my_car.actual_path:
        actual_path_point = coord_global_to_frame(point)
        draw_point(canvas, GREEN, actual_path_point)

    # Draw the car
    pygame.draw.polygon(canvas, GREEN, my_car.corners())
    # Front indicator
    pygame.draw.circle(canvas, RED, my_car.front_point(), int(0.1 * PIX_PER_FOOT))
    # Draw intended path
    if control_mode == 'rectangle_execution':
        rect_points = [coord_global_to_frame(point) for point in my_car.rect_points]
        pygame.draw.polygon(canvas, BLUE, rect_points, 1)
    my_car.update()

    myfont1 = pygame.font.SysFont(None, 30)
    label1 = myfont1.render("X Position (ft): " + str(round(my_car.xpos_actual, 2)), 1, RED)
    label2 = myfont1.render("Y Position (ft): " + str(round(my_car.ypos_actual, 2)), 1, RED)
    label3 = myfont1.render("Yaw (deg): " + str(round(my_car.yaw, 2)), 1, RED)
    label4 = myfont1.render("X Vel (ft/s): " + str(round(my_car.x_vel, 2)), 1, RED)
    label5 = myfont1.render("Y Vel (ft/s): " + str(round(my_car.y_vel, 2)), 1, RED)
    label6 = myfont1.render("Angular Vel (deg/s): " + str(round(my_car.r_vel, 2)), 1, RED)
    label7 = myfont1.render("Psi_1 (rad/s): " + str(round(my_car.psi_1, 2)), 1, RED)
    label8 = myfont1.render("Psi_2 (rad/s): " + str(round(my_car.psi_2, 2)), 1, RED)
    label9 = myfont1.render("Psi_3 (rad/s): " + str(round(my_car.psi_3, 2)), 1, RED)
    label10 = myfont1.render("Psi_4 (rad/s): " + str(round(my_car.psi_4, 2)), 1, RED)
    label11 = myfont1.render("Control mode: " + control_mode, 1, RED)
    canvas.blit(label1, (50, 20))
    canvas.blit(label2, (50, 40))
    canvas.blit(label3, (50, 60))
    canvas.blit(label4, (50, 80))
    canvas.blit(label5, (50, 100))
    canvas.blit(label6, (50, 120))
    canvas.blit(label7, (50, 140))
    canvas.blit(label8, (50, 160))
    canvas.blit(label9, (50, 180))
    canvas.blit(label10, (50, 200))
    canvas.blit(label11, (50, 220))


def keydown(event):
    global my_car, control_mode
    mods = pygame.key.get_mods()

    if 'manual' in control_mode:
        my_car.stop()
        control_mode = ''
        return
    if event.type == pygame.MOUSEBUTTONUP:
        return
    if event.key == pygame.K_r and mods & pygame.KMOD_CTRL:
        reset()


def keyup(event):
    global paddle1_vel, paddle2_vel

    if event.key in (pygame.K_w, pygame.K_s):
        pass
    elif event.key in (pygame.K_UP, pygame.K_DOWN):
        pass

app = QApplication(sys.argv)

ex = RobotMenu()
ex.setWindowTitle('Simulation Menu')

init()


while True:

    draw(window)

    for event in pygame.event.get():

        if event.type == pygame.KEYDOWN:
            keydown(event)
        elif event.type == pygame.KEYUP:
            keyup(event)
        elif event.type == pygame.MOUSEBUTTONUP:
            keydown(event)
        elif event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

    pygame.display.update()
    fps.tick_busy_loop(FRAMERATE)