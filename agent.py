'''An agent with Seek, Flee, Arrive, Pursuit behaviours

Created for COS30002 AI for Games by Clinton Woodward cwoodward@swin.edu.au

'''

from vector2d import Vector2D
from vector2d import Point2D
from graphics import egi, KEY
from math import sin, cos, radians
from random import random, randrange, uniform
from path import Path


class Agent(object):
    def __init__(self, world=None):
        # keep a reference to the world object
        self.world = world
        if len(self.world.agents) > 0:
            self.mode = 'prey'
        else:
            self.mode = 'hunt'
        # where am i and where am i going? random
        dir = radians(random()*360)
        self.pos = Vector2D(randrange(world.cx), randrange(world.cy))
        self.vel = Vector2D()
        self.heading = Vector2D(sin(dir), cos(dir))
        self.side = self.heading.perp()
        if self.mode == 'hunt':
            scale = 30.0
            self.max_speed = 5.0 * scale
            self.max_force = 250.0
            self.mass = 2.0
        else:
            scale = 20.0
            self.max_speed = 15.0 * scale
            self.max_force = 500.0
            self.mass = 0.75
        self.scale = Vector2D(scale, scale)  # easy scaling of agent size
        self.force = Vector2D()  # current steering force
        self.acceeration = Vector2D()  # current steering force
        # data for drawing this agent
        self.color = 'ORANGE'
        self.vehicle_shape = [
            Point2D(-1.0,  0.6),
            Point2D( 1.0,  0.0),
            Point2D(-1.0, -0.6)
        ]
        self.show_info = False
        self.wander_target = Vector2D(1, 0)
        self.wander_dist = 1.0 * scale
        self.wander_radius = 1.0 * scale
        self.wander_jitter = 10.0 * scale
        self.bRadius = scale
        self.hiding_spots = []
        self.best_spot = None

    def calculate(self, delta):
        # reset the steering force
        mode = self.mode
        if mode == 'hunt':
            force = self.hunt(delta)
        elif mode == 'prey':
            force = self.prey()
        force.x /= self.mass
        force.y /= self.mass
        self.force = force
        return force
    
    def update(self, delta):
        ''' update vehicle position and orientation '''
        force = self.calculate(delta)
        force.truncate(self.max_force)
        # new velocity
        self.vel += force * delta
        for obst in self.world.obstacles:
            if self.flee(obst.coords, 50) != Vector2D(0, 0):
                self.vel = self.flee(obst.coords, 50) * delta * 20
        # check for limits of new velocity
        self.vel.truncate(self.max_speed)
        # update position
        self.pos += self.vel * delta
        # update heading is non-zero velocity (moving)
        if self.vel.length_sq() > 0.00000001:
            self.heading = self.vel.get_normalised()
            self.side = self.heading.perp()
        # treat world as continuous space - wrap new position if needed
        self.world.wrap_around(self.pos)

    def render(self, color=None):
        ''' Draw the triangle agent with color'''
        egi.set_pen_color(name=self.color)
        pts = self.world.transform_points(self.vehicle_shape, self.pos,
                                          self.heading, self.side, self.scale)

        if (self.mode == 'hunt'):
            wnd_pos = Vector2D(self.wander_dist, 0)
            wld_pos = self.world.transform_point(wnd_pos, self.pos, self.heading, self.side)
            egi.green_pen()
            egi.circle(wld_pos, self.wander_radius)
            egi.red_pen()
            wnd_pos = (self.wander_target + Vector2D(self.wander_dist, 0))
            wld_pos = self.world.transform_point(wnd_pos, self.pos, self.heading, self.side)
            egi.circle(wld_pos, 3)
        # draw it!
        egi.closed_shape(pts)

        if self.show_info:
            s = 0.5 # <-- scaling factor
            # force
            egi.red_pen()
            egi.line_with_arrow(self.pos, self.pos + self.force * s, 5)
            # velocity
            egi.grey_pen()
            egi.line_with_arrow(self.pos, self.pos + self.vel * s, 5)
            # net (desired) change
            egi.white_pen()
            egi.line_with_arrow(self.pos+self.vel * s, self.pos+ (self.force+self.vel) * s, 5)
            egi.line_with_arrow(self.pos, self.pos+ (self.force+self.vel) * s, 5)

    def speed(self):
        return self.vel.length()

    #--------------------------------------------------------------------------

    def seek(self, target_pos):
        ''' move towards target position '''
        desired_vel = (target_pos - self.pos).normalise() * self.max_speed
        return (desired_vel - self.vel)

    def flee(self, hunter_pos, set_dist):
        ''' move away from hunter position '''
        flee_dist = set_dist
        if ((self.pos.x < hunter_pos.x + flee_dist) and (self.pos.x > hunter_pos.x - flee_dist)):
            if ((self.pos.y < hunter_pos.y + flee_dist) and (self.pos.y > hunter_pos.y - flee_dist)):
                desired_vel = (self.pos - hunter_pos).normalise() * self.max_speed
                return (desired_vel - self.vel)
        return Vector2D()
                
    def wander(self, delta):
        wt = self.wander_target
        jitter_tts = self.wander_jitter * delta
        wt += Vector2D(uniform(-1, 1) * jitter_tts, uniform(-1, 1) * jitter_tts)
        wt.normalise()
        wt *= self.wander_radius
        target = wt + Vector2D(self.wander_dist, 0)
        wld_target = self.world.transform_point(target, self.pos, self.heading, self.side)
        return self.seek(wld_target)

    def hunt(self, delta):
        for obst in self.world.obstacles:
            dist_from_x = obst.coords.x - self.pos.x
            dist_from_y = obst.coords.y - self.pos.y
            if dist_from_x > 0:
                hiding_x = obst.coords.x + obst.radius + 25
            else:
                hiding_x = obst.coords.x - obst.radius - 25
            if dist_from_y > 0:
                hiding_y = obst.coords.y + obst.radius + 25
            else:
                hiding_y = obst.coords.y - obst.radius - 25
            self.hiding_spots.append(Vector2D(hiding_x, hiding_y))
        return self.wander(delta)
            
    def prey(self):
        for agent in self.world.agents:
            if agent.mode == 'hunt':
                self.hiding_spots = agent.hiding_spots
                hunter = agent
        if self.pos.distance(hunter.pos) < 100:
            return self.flee(hunter.pos, 100)
        best_spot = self.hiding_spots[0]
        for spot in self.hiding_spots:
            my_dist = self.pos.distance(spot)
            hunter_dist = hunter.pos.distance(spot)
            spot_value = hunter_dist - my_dist
            best_my_dist = self.pos.distance(best_spot)
            best_hunter_dist = hunter.pos.distance(best_spot)
            best_value = best_hunter_dist - best_my_dist
            if spot_value > best_value:
                best_spot = spot
        self.best_spot = best_spot
        return self.seek(best_spot)
