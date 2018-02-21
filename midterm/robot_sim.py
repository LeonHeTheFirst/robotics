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
PIXELS_PER_FOOT = 40
WIDTH = 30 * PIXELS_PER_FOOT
HEIGHT = 15 * PIXELS_PER_FOOT
BALL_RADIUS = 20
PAD_WIDTH = 8
PAD_HEIGHT = 80
HALF_PAD_WIDTH = PAD_WIDTH // 2
HALF_PAD_HEIGHT = PAD_HEIGHT // 2
robot_pos = [0, 0]
ball_vel = [0, 0]
paddle1_vel = 0
paddle2_vel = 0
l_score = 0
r_score = 0

class Car():
    def __init__(self, xpos, ypos, yaw=0):
        # width = 1ft
        # length = 2ft
        self.xpos = xpos
        self.ypos = ypos
        self.yaw = yaw
        self.x_vel = 0 # ft/s
        self.y_vel = 0 # ft/s
        self.r_vel = 0 # deg/s
        # self.surface = pygame.Surface((width, height))

    def corners(self):
        yaw_rad = math.radians(self.yaw)
        p1_y = self.ypos + (math.cos(yaw_rad) - 2 * math.sin(yaw_rad)) * PIXELS_PER_FOOT
        p2_y = self.ypos + (-1 * math.cos(yaw_rad) - 2 * math.sin(yaw_rad)) * PIXELS_PER_FOOT
        p3_y = self.ypos + (-1 * math.cos(yaw_rad) + 2 * math.sin(yaw_rad)) * PIXELS_PER_FOOT
        p4_y = self.ypos + (math.cos(yaw_rad) + 2 * math.sin(yaw_rad)) * PIXELS_PER_FOOT
        p1_x = self.xpos + (2 * math.cos(yaw_rad) + math.sin(yaw_rad)) * PIXELS_PER_FOOT
        p2_x = self.xpos + (2 * math.cos(yaw_rad) - math.sin(yaw_rad)) * PIXELS_PER_FOOT
        p3_x = self.xpos + (-2 * math.cos(yaw_rad) - math.sin(yaw_rad)) * PIXELS_PER_FOOT
        p4_x = self.xpos + (-2 * math.cos(yaw_rad) + math.sin(yaw_rad)) * PIXELS_PER_FOOT
        # print('point 1: ', p1_x, ',', p1_y)
        # print('point 2: ', p2_x, ',', p2_y)
        # print('point 3: ', p3_x, ',', p3_y)
        # print('point 4: ', p4_x, ',', p4_y)
        return [[p1_x, p1_y], [p2_x, p2_y], [p3_x, p3_y], [p4_x, p4_y]]

    def front_point(self):
        yaw_rad = math.radians(self.yaw)
        p_x = int(self.xpos + (1.8 * math.cos(yaw_rad)) * PIXELS_PER_FOOT)
        p_y = int(self.ypos - (1.8 * math.sin(yaw_rad)) * PIXELS_PER_FOOT)
        return [p_x, p_y]

    def position(self):
        return (self.xpos, self.ypos)

    def set_position(self, xpos, ypos):
        self.xpos = xpos
        self.ypos = ypos

    def update(self):
        self.xpos += self.x_vel * PIXELS_PER_FOOT * (1 / FRAMERATE)
        self.ypos += self.y_vel * PIXELS_PER_FOOT * (1 / FRAMERATE)
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

my_car = Car(200, 200)
my_car.x_vel = 1
my_car.y_vel = 1
my_car.r_vel = 1

window = pygame.display.set_mode((WIDTH, HEIGHT), 0, 32)
pygame.display.set_caption('Robotics Simulation')


def robot_init(right):
    global robot_pos, ball_vel
    robot_pos = [WIDTH // 2, HEIGHT // 2]
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
    my_car.set_position(WIDTH/2, HEIGHT/2)
    my_car.yaw = 0


def draw(canvas):
    global paddle1_pos, paddle2_pos, robot_pos, ball_vel, l_score, r_score, my_car

    canvas.fill(BLACK)

    # robot_pos[0] += int(ball_vel[0])
    # robot_pos[1] += int(ball_vel[1])
    # 6X6 Grid
    margin = int(0.5 * PIXELS_PER_FOOT)
    for column in range(0, WIDTH, margin):
        for row in range(0, HEIGHT, margin):
            pygame.draw.rect(canvas, WHITE, [column,row,WIDTH,HEIGHT], 1)
    # Body of car
    # pygame.draw.rect(canvas, WHITE, (robot_pos, (10,10)), 0)
    pygame.draw.polygon(canvas, GREEN, my_car.corners())
    # Front indicator
    pygame.draw.circle(canvas, RED, my_car.front_point(), int(0.1 * PIXELS_PER_FOOT))
    # my_car.rotate(5)
    # my_car.yaw += 0.5

    my_car.update()
    # rot_surface = my_car.rot_center()
    # print(rot_surface)
    # canvas.blit(rot_surface, my_car.position())

    # Wheels
    # pygame.draw.rect(canvas, WHITE, (robot_pos, (20,10)), 0)
    # pygame.draw.rect(canvas, WHITE, (robot_pos, (20,10)), 0)
    # pygame.draw.rect(canvas, WHITE, (robot_pos, (20,10)), 0)
    # pygame.draw.rect(canvas, WHITE, (robot_pos, (20,10)), 0)


    myfont1 = pygame.font.SysFont(None, 20)
    label1 = myfont1.render("X Position " + str(my_car.xpos), 1, (255, 255, 0))
    label2 = myfont1.render("X Position " + str(my_car.ypos), 1, (255, 255, 0))
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