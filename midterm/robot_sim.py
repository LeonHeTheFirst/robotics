#!/usr/bin/env python3

import random
import pygame
import sys
import math

pygame.init()
fps = pygame.time.Clock()


WHITE = (255, 255, 255)
ORANGE = (255,140,0)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
BLACK = (0, 0, 0)

FRAMERATE = 60
PIX_PER_FOOT = 40
CAR_LENGTH = 4
CAR_SCREEN_WIDTH = 2
CAR_HALF_LENGTH = 2
CAR_HALF_SCREEN_WIDTH = 1
SCREEN_WIDTH = 30 * PIX_PER_FOOT
SCREEN_HEIGHT = 15 * PIX_PER_FOOT
SCREEN_BUFFER = 2 * PIX_PER_FOOT
robot_pos = [0, 0]
ball_vel = [0, 0]
paddle1_vel = 0
paddle2_vel = 0
l_score = 0
r_score = 0

class Car():
    def __init__(self, xpos, ypos, yaw=0):
        # SCREEN_WIDTH = 1ft
        # length = 2ft
        self.xpos = xpos
        self.ypos = ypos
        self.xpos_actual = xpos / PIX_PER_FOOT
        self.ypos_actual = (SCREEN_HEIGHT - ypos) / PIX_PER_FOOT
        self.yaw = yaw
        self.phi_1 = 0
        self.phi_2 = 0
        self.phi_3 = 0
        self.phi_4 = 0
        self.x_vel = 0 # ft/s
        self.y_vel = 0 # ft/s
        self.r_vel = 0 # deg/s
        # self.surface = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT))

    def corners(self):
        yaw_rad = math.radians(self.yaw)
        cos_yaw = math.cos(yaw_rad)
        sin_yaw = math.sin(yaw_rad)
        p1_x = self.xpos + (-CAR_HALF_SCREEN_WIDTH * cos_yaw + CAR_HALF_LENGTH * sin_yaw) * PIX_PER_FOOT
        p2_x = self.xpos + (CAR_HALF_SCREEN_WIDTH * cos_yaw + CAR_HALF_LENGTH * sin_yaw) * PIX_PER_FOOT
        p3_x = self.xpos + (-CAR_HALF_SCREEN_WIDTH * cos_yaw - CAR_HALF_LENGTH * sin_yaw) * PIX_PER_FOOT
        p4_x = self.xpos + (CAR_HALF_SCREEN_WIDTH * cos_yaw - CAR_HALF_LENGTH * sin_yaw) * PIX_PER_FOOT
        p1_y = self.ypos + (CAR_HALF_LENGTH * cos_yaw + CAR_HALF_SCREEN_WIDTH * sin_yaw) * PIX_PER_FOOT
        p2_y = self.ypos + (CAR_HALF_LENGTH * cos_yaw - CAR_HALF_SCREEN_WIDTH * sin_yaw) * PIX_PER_FOOT
        p3_y = self.ypos + (-CAR_HALF_LENGTH * cos_yaw + CAR_HALF_SCREEN_WIDTH * sin_yaw) * PIX_PER_FOOT
        p4_y = self.ypos + (-CAR_HALF_LENGTH * cos_yaw - CAR_HALF_SCREEN_WIDTH * sin_yaw) * PIX_PER_FOOT
        # print('point 1: ', p1_x, ',', p1_y)
        # print('point 2: ', p2_x, ',', p2_y)
        # print('point 3: ', p3_x, ',', p3_y)
        # print('point 4: ', p4_x, ',', p4_y)
        return [[p2_x, p2_y], [p1_x, p1_y], [p3_x, p3_y], [p4_x, p4_y]]

    def front_point(self):
        yaw_rad = math.radians(self.yaw)
        p_y = int(self.ypos - (1.8 * math.cos(yaw_rad)) * PIX_PER_FOOT)
        p_x = int(self.xpos - (1.8 * math.sin(yaw_rad)) * PIX_PER_FOOT)
        # print('xpos: ', self.xpos)
        # print('ypos: ', self.ypos)
        # print('cos: ', math.cos(yaw_rad))
        # print('sin: ', math.sin(yaw_rad))
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

    # def draw(self, canvas):
    #     # self.surface.fill(ORANGE)
    #     # self.surface.blit(self, (self.xpos, self.ypos))

    # def rotated_surface(self):
    #     """rotate an image while keeping its center and size"""
    #     orig_rect = self.surface.get_rect()
    #     rot_image = pygame.transform.rotate(self.surface, self.yaw)
    #     rot_rect = orig_rect.copy()
    #     rot_rect.center = rot_image.get_rect().center
    #     rot_image = rot_image.subsurface(rot_rect).copy()
    #     rot_image.fill(ORANGE)
    #     return rot_image

    # def rot_center(self):
    #     """rotate a Surface, maintaining position."""

    #     # loc = self.surface.get_rect().center
    #     # rot_sprite = pygame.transform.rotate(self.surface, self.yaw)
    #     rot_sprite.get_rect().center = loc
    #     rot_sprite.fill(ORANGE)
    #     return rot_sprite

my_car = Car(SCREEN_WIDTH/2, SCREEN_HEIGHT/2)
my_car.x_vel = 10
my_car.y_vel = 10
my_car.r_vel = 10

window = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), 0, 32)
pygame.display.set_caption('Robotics Simulation')


def robot_init(right):
    global robot_pos, ball_vel
    robot_pos = [SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2]
    horz = random.randrange(2, 4)
    vert = random.randrange(1, 3)

    if right == False:
        horz = - horz

    ball_vel = [horz, -vert]


def init():
    if random.randrange(0, 2) == 0:
        robot_init(True)
    else:
        robot_init(False)

def reset():
    global my_car
    my_car.set_position(SCREEN_WIDTH/2, SCREEN_HEIGHT/2)
    my_car.yaw = 0


def draw(canvas):
    global paddle1_pos, paddle2_pos, robot_pos, ball_vel, l_score, r_score, my_car

    canvas.fill(BLACK)

    # robot_pos[0] += int(ball_vel[0])
    # robot_pos[1] += int(ball_vel[1])
    # 6X6 Grid
    margin = int(0.5 * PIX_PER_FOOT)
    for column in range(0, SCREEN_WIDTH, margin):
        for row in range(0, SCREEN_HEIGHT, margin):
            pygame.draw.rect(canvas, WHITE, [column,row,SCREEN_WIDTH,SCREEN_HEIGHT], 1)
    # Body of car
    # pygame.draw.rect(canvas, WHITE, (robot_pos, (10,10)), 0)
    pygame.draw.polygon(canvas, GREEN, my_car.corners())
    # Front indicator
    pygame.draw.circle(canvas, RED, my_car.front_point(), int(0.1 * PIX_PER_FOOT))
    # my_car.rotate(5)
    # my_car.yaw += 0.5

    my_car.update()
    # rot_surface = my_car.rot_center()
    # print(rot_surface)
    # canvas.blit(rot_surface, my_car.position())

    myfont1 = pygame.font.SysFont(None, 20)
    label1 = myfont1.render("X Position " + str(my_car.xpos_actual), 1, (255, 255, 0))
    label2 = myfont1.render("Y Position " + str(my_car.ypos_actual), 1, (255, 255, 0))
    label3 = myfont1.render("Yaw " + str(my_car.yaw), 1, (255, 255, 0))
    canvas.blit(label1, (50, 20))
    canvas.blit(label2, (50, 30))
    canvas.blit(label3, (50, 40))


def keydown(event):
    global paddle1_vel, paddle2_vel

    if event.key == pygame.K_r:
        reset()


def keyup(event):
    global paddle1_vel, paddle2_vel

    if event.key in (pygame.K_w, pygame.K_s):
        paddle1_vel = 0
    elif event.key in (pygame.K_UP, pygame.K_DOWN):
        paddle2_vel = 0


init()


while True:

    draw(window)

    for event in pygame.event.get():

        if event.type == pygame.KEYDOWN:
            keydown(event)
        elif event.type == pygame.KEYUP:
            keyup(event)
        elif event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

    pygame.display.update()
    fps.tick_busy_loop(FRAMERATE)