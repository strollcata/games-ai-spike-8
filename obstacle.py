from vector2d import Vector2D
from matrix33 import Matrix33
from graphics import egi
from random import random, randrange, uniform


class Obstacle(object):
    def __init__(self, world = None):
        self.world = world
        self.radius = 50
        self.sep_radius = 0
        while self.sep_radius <= self.radius + 25:
            self.sep_radius += 25
        open_coords = []
        for y_incr in range(75, world.cy - 50, 25):
            for x_incr in range(75, world.cx - 50, 25):
                too_close = False
                for agent in world.agents:
                    if ((agent.pos.x > x_incr - self.sep_radius) and (agent.pos.x < x_incr + self.sep_radius)):
                        if ((agent.pos.y > y_incr - self.sep_radius) and (agent.pos.y < y_incr + self.sep_radius)):
                            too_close = True
                for obst in world.obstacles:
                    if ((obst.coords.x > x_incr - self.sep_radius) and (obst.coords.x < x_incr + self.sep_radius)):
                        if ((obst.coords.y > y_incr - self.sep_radius) and (obst.coords.y < y_incr + self.sep_radius)):
                            too_close = True
                if too_close:
                    continue
                else:
                    open_coords.append([x_incr, y_incr])
        if len(open_coords) > 0:
            rand_select = randrange(0, len(open_coords))
            my_x = open_coords[rand_select][0]
            my_y = open_coords[rand_select][1]
            self.coords = Vector2D(my_x, my_y)
        else:
            self.coords = False

    def render(self):
        egi.blue_pen()
        egi.circle(self.coords, self.radius)
