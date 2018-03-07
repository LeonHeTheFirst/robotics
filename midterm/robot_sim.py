#!/usr/bin/env python3

from PyQt5.QtWidgets import QWidget, QPushButton, QApplication, QLabel, QLineEdit
# from PyQt5.QtGui import QColor
import math
import pygame
import random
import sys

pygame.init()
fps = pygame.time.Clock()


WHITE = (255, 255, 255)
ORANGE = (255,140,0)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
YELLOW = (255, 255, 0)
BLACK = (0, 0, 0)

FRAMERATE = 60
PIX_PER_FOOT = 40
CAR_LENGTH = 4
CAR_WIDTH = 2
CAR_HALF_LENGTH = 2
CAR_HALF_WIDTH = 1
WHEEL_RADIUS = 1
SCREEN_WIDTH = 30 * PIX_PER_FOOT
SCREEN_HEIGHT = 15 * PIX_PER_FOOT
SCREEN_BUFFER = 2 * PIX_PER_FOOT
robot_pos = [0, 0]
ball_vel = [0, 0]
l_score = 0
r_score = 0
control_mode = ''


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
        self.set_position(SCREEN_WIDTH/2, SCREEN_HEIGHT/2)

    def update(self):
        self.xpos += self.x_vel * PIX_PER_FOOT * (1 / FRAMERATE)
        self.ypos -= self.y_vel * PIX_PER_FOOT * (1 / FRAMERATE)
        if self.xpos < SCREEN_BUFFER or self.xpos > SCREEN_WIDTH - SCREEN_BUFFER:
            self.reset_screen_pos()
        if self.ypos < SCREEN_BUFFER or self.ypos > SCREEN_HEIGHT - SCREEN_BUFFER:
            self.reset_screen_pos()
        self.xpos_actual += self.x_vel * (1 / FRAMERATE)
        self.ypos_actual += self.y_vel * (1 / FRAMERATE)
        self.yaw += self.r_vel * (1 / FRAMERATE)
        if self.yaw > 360 or self.yaw < 0:
            self.yaw %= 360
        self.find_psi_from_desired_vel_cartesian(self.x_vel, self.y_vel, self.r_vel)

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

        # Specify Point to Travel To

        # Specify Path to Travel On

        self.setGeometry(200, 200, 500, 500)
        self.setWindowTitle('Robot Menu')
        self.show()

    def setPsi(self, pressed):
        global my_car, control_mode
        my_car.psi_1 = str_to_float(self.psi_1_field.text())
        my_car.psi_2 = str_to_float(self.psi_2_field.text())
        my_car.psi_3 = str_to_float(self.psi_3_field.text())
        my_car.psi_4 = str_to_float(self.psi_4_field.text())
        my_car.find_vel_from_psi()
        control_mode = 'manual'

    def setVel(self, pressed):
        global my_car, control_mode
        my_car.x_vel = str_to_float(self.x_vel_field.text())
        my_car.y_vel = str_to_float(self.y_vel_field.text())
        my_car.r_vel = str_to_float(self.r_vel_field.text())
        my_car.find_psi_from_desired_vel_cartesian(my_car.x_vel, my_car.y_vel, my_car.r_vel)
        my_car.find_vel_from_psi()
        control_mode = 'manual'

    def setPolarVel(self, pressed):
        global my_car, control_mode
        speed = str_to_float(self.speed_field.text())
        direction = str_to_float(self.direction_field.text())
        my_car.r_vel = str_to_float(self.r_vel_field.text())
        my_car.find_psi_from_desired_vel_polar(speed, direction)
        my_car.find_vel_from_psi()
        control_mode = 'manual'

    def setDest(self):
        pass

    def setPath(self):
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
    global my_car
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

def draw(canvas):
    global robot_pos, ball_vel, l_score, r_score, my_car

    canvas.fill(BLACK)

    # 6X6 Grid
    margin = int(0.5 * PIX_PER_FOOT)
    for column in range(0, SCREEN_WIDTH, margin):
        for row in range(0, SCREEN_HEIGHT, margin):
            pygame.draw.rect(canvas, WHITE, [column,row,SCREEN_WIDTH,SCREEN_HEIGHT], 1)
    # Draw the car
    pygame.draw.polygon(canvas, GREEN, my_car.corners())
    # Front indicator
    pygame.draw.circle(canvas, RED, my_car.front_point(), int(0.1 * PIX_PER_FOOT))

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


def keydown(event):
    global my_car, control_mode
    mods = pygame.key.get_mods()

    if control_mode == 'manual':
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