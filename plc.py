#!/usr/bin/env python

#########################################
# Imports
#########################################
# - Logging
import logging

# - Multithreading
from twisted.internet import reactor

# - Modbus
from pymodbus.server.asynchronous import StartTcpServer, StopServer
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSequentialDataBlock
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
from pymodbus.transaction import ModbusRtuFramer, ModbusAsciiFramer

# - World Simulator
import sys, random, time, traceback, threading
import pygame
from pygame.locals import *
from pygame.color import *
import pymunk
import pymunk.pygame_util
from pymunk import Vec2d

#########################################
# inet conf
#########################################
MODBUS_SERVER_IP = "localhost" ##
MODBUS_SERVER_PORT = 502 ##

#########################################
# Logging
#########################################
logging.basicConfig()
log = logging.getLogger()
log.setLevel(logging.INFO)

#########################################
# Util Functions
#########################################
def PLCSetTag(addr, value):
    context[0x0].setValues(3, addr, [value])

def PLCGetTag(addr):
    return context[0x0].getValues(3, addr, count=1)[0]

#########################################
# World Code
#########################################

# "Constants"
SCREEN_WIDTH = 600
SCREEN_HEIGHT = 350
FPS=50.0

PLC_TAG_LEVEL_SENSOR = 0x1
PLC_TAG_LIMIT_SWITCH = 0x2
PLC_TAG_MOTOR = 0x3
PLC_TAG_NOZZLE = 0x4
PLC_TAG_RUN = 0x10


# Global Variables
global bottles
bottles = []


def to_pygame(p):
    """Small hack to convert pymunk to pygame coordinates"""
    return int(p.x), int(-p.y+600)

# Shape functions

def add_ball(space):
    mass = 0.01
    radius = 3
    inertia = pymunk.moment_for_circle(mass, 0, radius, (0,0))
    body = pymunk.Body(mass, inertia)

    x = random.randint(181,182)
    body.position = x, 400
    shape = pymunk.Circle(body, radius, (0,0))
    shape.collision_type = 0x5 #liquid
    space.add(body, shape)
    return shape

def draw_ball(screen, ball, color=THECOLORS['blue']):
    p = int(ball.body.position.x), 600-int(ball.body.position.y)
    pygame.draw.circle(screen, color, p, int(ball.radius), 2)

def add_bottle_in_sensor(space):
    body = pymunk.Body()
    body.position = (40, 300)
    radius = 5
    shape = pymunk.Circle(body, radius, (0, 0))
    shape.collision_type = 0x7 # 'bottle_in'
    space.add(shape)
    return shape

def add_level_sensor(space):
    body = pymunk.Body()
    body.position = (155, 380)
    radius = 5
    shape = pymunk.Circle(body, radius, (0, 0))
    shape.collision_type = 0x4 # level_sensor
    space.add(shape)
    return shape

def add_limit_switch(space):
    body = pymunk.Body()
    body.position = (200, 300)
    radius = 5
    shape = pymunk.Circle(body, radius, (0, 0))
    shape.collision_type = 0x1 # switch
    space.add(shape)
    return shape

def add_nozzle(space):
    body = pymunk.Body()
    body.position = (180, 430)
    vs = [(170, 400),(170, 430),(190, 400),(190, 430)]
    shape = pymunk.Poly(body, vs)
    space.add(shape)
    return shape

def add_base(space):
    global SCREEN_WIDTH
    global SCREEN_HEIGHT

    body = pymunk.Body(10, 100, body_type=pymunk.Body.KINEMATIC)
    body.position = (0, SCREEN_HEIGHT-50)
    vs = [(0, SCREEN_HEIGHT-90),(0, SCREEN_HEIGHT-50),(SCREEN_WIDTH, SCREEN_HEIGHT-90),(SCREEN_WIDTH, SCREEN_HEIGHT-50)]
    shape_box = pymunk.Poly(body, vs)

    body.position = (0, SCREEN_HEIGHT-50)
    shape = pymunk.Segment(body, (0, 0), (SCREEN_WIDTH, 0), 2.0)

    shape.friction = 1.0
    shape.elasticity = 0.95
    shape.collision_type = 0x19 # base
    space.add(shape, shape_box)

    return shape, shape_box

def add_bottle(space):
    mass = 10
    inertia = 0xFFFFFFFFF #7*16
    body = pymunk.Body(mass, inertia, pymunk.Body.KINEMATIC)
    body.position = (130,301)
    body.collision_type = 0x15
    l1 = pymunk.Segment(body, (-150, 0), (-100, 0), 2.0)
    l2 = pymunk.Segment(body, (-150, 0), (-150, 100), 2.0)
    l3 = pymunk.Segment(body, (-100, 0), (-100, 100), 2.0)

    # Glass friction
    l1.friction = 0.94
    l2.friction = 0.94
    l3.friction = 0.94

    # Set collision types for sensors
    l1.collision_type = 0x2 # bottle_bottom
    l2.collision_type = 0x3 # bottle_side
    l3.collision_type = 0x3 # bottle_side

    space.add(l1,l2,l3)
    return l1,l2,l3, body

def draw_polygon(screen, shape):
    points = shape.get_vertices()
    fpoints = []
    for p in points:
        fpoints.append(to_pygame(p))
    pygame.draw.polygon(screen, THECOLORS['black'], fpoints)


def draw_lines(screen, lines, color=THECOLORS['dodgerblue4']):
    for line in lines[:3]:
        body = line.body
        pv1 = body.position + line.a.rotated(body.angle)
        pv2 = body.position + line.b.rotated(body.angle)
        p1 = to_pygame(pv1)
        p2 = to_pygame(pv2)
        pygame.draw.lines(screen, color, False, [p1,p2])

# Collision handlers
def no_collision(space, arbiter, *args, **kwargs):
    return False

def level_ok(space, arbiter, *args, **kwargs):
    log.debug("Level reached")
    PLCSetTag(PLC_TAG_LIMIT_SWITCH, 0) # Limit Switch Release, Fill Bottle
    PLCSetTag(PLC_TAG_LEVEL_SENSOR, 1) # Level Sensor Hit, Bottle Filled
    PLCSetTag(PLC_TAG_NOZZLE, 0) # Close nozzle
    return False

def bottle_in_place(arbiter, space, *args, **kwargs):
    log.debug("Bottle in place")
    PLCSetTag(PLC_TAG_LIMIT_SWITCH, 1)
    PLCSetTag(PLC_TAG_LEVEL_SENSOR, 0)
    PLCSetTag(PLC_TAG_NOZZLE, 1) # Open nozzle

    return False

def add_new_bottle(arbiter, space, *args, **kwargs):
    bottles.append(add_bottle(space))
    log.debug("Adding new bottle")
    return False

def runWorld():

    log.info("RUN")

    global SCREEN_WIDTH
    global SCREEN_HEIGHT

    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.RESIZABLE)
    pygame.display.set_caption("IO ICS\SCADA")
    clock = pygame.time.Clock()
    running = True

    space = pymunk.Space()
    space.gravity = (0.0, -900.0)

    # # Limit switch with bottle bottom
    space.add_collision_handler(0x1, 0x2).begin=bottle_in_place
    # # Level sensor with water
    space.add_collision_handler(0x4, 0x5).begin=level_ok

    # Level sensor with ground
    space.add_collision_handler(0x4, 0x6).begin=no_collision
    # Limit switch with ground
    space.add_collision_handler(0x1, 0x6).begin=no_collision
    # Limit switch with bottle side
    space.add_collision_handler(0x1, 0x3).begin=no_collision
    # Level sensor with bottle side
    space.add_collision_handler(0x4, 0x3).begin=no_collision
    # Bottle in with bottle sides and bottom
    space.add_collision_handler(0x7, 0x2).begin=no_collision
    space.add_collision_handler(0x7, 0x2).separate=add_new_bottle

    space.add_collision_handler(0x7, 0x3).begin=no_collision


    base, base_box = add_base(space)
    nozzle = add_nozzle(space)
    limit_switch = add_limit_switch(space)
    level_sensor = add_level_sensor(space)
    bottle_in = add_bottle_in_sensor(space)

    global bottles
    bottles.append(add_bottle(space))

    balls = []

    ticks_to_next_ball = 1

    fontBig = pygame.font.SysFont(None, 40)
    fontMedium = pygame.font.SysFont(None, 26)
    fontSmall = pygame.font.SysFont(None, 18)

    while running:
        clock.tick(FPS)

        for event in pygame.event.get():
            if event.type == QUIT:
                running = False
            elif event.type == KEYDOWN and event.key == K_ESCAPE:
                running = False

            if event.type == pygame.VIDEORESIZE:
                # There's some code to add back window content here.
                SCREEN_WIDTH = event.w
                screen = pygame.display.set_mode((event.w, SCREEN_HEIGHT),
                                                  pygame.RESIZABLE)

                base, base_box = add_base(space)

        screen.fill(THECOLORS["white"]) #
        foo = pygame.image.load('Logo.png').convert() # setting the background image
        screen.blit(foo, ( 0,0))

        if PLCGetTag(PLC_TAG_RUN): # if the plant working
            # Motor Logic
            if (PLCGetTag(PLC_TAG_LIMIT_SWITCH) == 1):
                PLCSetTag(PLC_TAG_MOTOR, 0)

            if (PLCGetTag(PLC_TAG_LEVEL_SENSOR) == 1):
                PLCSetTag(PLC_TAG_MOTOR, 1)

            ticks_to_next_ball -= 1

            if not PLCGetTag(PLC_TAG_LIMIT_SWITCH):
                PLCSetTag(PLC_TAG_MOTOR, 1)

            if ticks_to_next_ball <= 0 and PLCGetTag(PLC_TAG_NOZZLE):
                ticks_to_next_ball = 1
                ball_shape = add_ball(space)
                balls.append(ball_shape)

            # Move the bottles
            if PLCGetTag(PLC_TAG_MOTOR) == 1:
                for bottle in bottles:
                    # bottle[0].body.position.x += 0.25
                    # bottle[0].body.position = Vec2d(bottle[0].body.position.x+1,
                    #                                 bottle[0].body.position.y)

                    bottle[3].position = Vec2d(bottle[3].position.x+0.25,
                                                bottle[3].position.y)

        # Draw water balls
        # Remove off-screen balls
        balls_to_remove = []
        for ball in balls:
            if ball.body.position.y < 150 or ball.body.position.x > SCREEN_WIDTH+150:
                balls_to_remove.append(ball)

            draw_ball(screen, ball)

        for ball in balls_to_remove:
            space.remove(ball, ball.body)
            balls.remove(ball)

        # Draw bottles
        for bottle in bottles:
            if bottle[0].body.position.x > SCREEN_WIDTH+150 or bottle[0].body.position.y < 150:
                # space.remove(bottle, bottle[0].body)
                bottles.remove(bottle)
                continue
            draw_lines(screen, bottle)

        # Draw the base and nozzle
        draw_polygon(screen, base_box)

        # Draw bouncing segment
        pv1 = base.body.position + base.a.rotated(base.body.angle)
        pv2 = base.body.position + base.b.rotated(base.body.angle)
        p1 = to_pygame(pv1)
        p2 = to_pygame(pv2)
        pygame.draw.lines(screen, THECOLORS['black'], False, [p1,p2])

        draw_polygon(screen, nozzle)
        # Draw the limit switch
        draw_ball(screen, limit_switch, THECOLORS['green'])
        # Draw the level sensor
        draw_ball(screen, level_sensor, THECOLORS['red'])

        title = fontMedium.render(str(""), 1, THECOLORS['deepskyblue'])
        name = fontBig.render(str(""), 1, THECOLORS['gray20'])
        instructions = fontSmall.render(str(""), 1, THECOLORS['gray'])
        screen.blit(title, (10, 40))
        screen.blit(name, (10, 10))
        screen.blit(instructions, (SCREEN_WIDTH-115, 10))

        space.step(1.0/FPS)
        pygame.display.flip()


    log.info("Done")

    # Stop reactor if running
    if reactor.running:
       reactor.callFromThread(reactor.stop)

#########################################
# Modbus Server Code
#########################################

store = ModbusSlaveContext(
    di = ModbusSequentialDataBlock(0, [0]*100),
    co = ModbusSequentialDataBlock(0, [0]*100),
    hr = ModbusSequentialDataBlock(0, [0]*100),
    ir = ModbusSequentialDataBlock(0, [0]*100))

context = ModbusServerContext(slaves=store, single=True)

identity = ModbusDeviceIdentification()
identity.VendorName  = 'IO ANALYTICA'
identity.ProductCode = 'IOMP'
identity.VendorUrl   = 'http://github.com/bashwork/pymodbus/'
identity.ProductName = 'IOPLC 3000'
identity.ModelName   = 'IOPLC Ultimate'
identity.MajorMinorRevision = '1.0'

context = ModbusServerContext(slaves=store, single=True)

def startModbusServer():
    StartTcpServer(context, identity=identity, address=(MODBUS_SERVER_IP, MODBUS_SERVER_PORT))

def main():
    reactor.callInThread(runWorld)
    startModbusServer()
    log.info("QUIT")
    return 0


if __name__ == '__main__':
    sys.exit(main())