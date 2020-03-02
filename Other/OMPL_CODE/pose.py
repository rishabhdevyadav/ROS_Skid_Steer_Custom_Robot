from __future__ import division
from numpy import cos, sin, degrees, radians

class Pose:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def __str__(self):
        return str(self.__dict__)

    def tolist(self):
        return [self.x, self.y, self.theta]

    def update(self, v, w, dt):
        self.x = self.x + v * cos(self.theta) * dt
        self.y = self.y + v * sin(self.theta) * dt
        self.theta = self.theta + w * dt
        self.theta = self.wrapAngle(self.theta)

    def wrapAngle(self, theta):
        if theta > radians(180):
            theta = theta - radians(360)
        if theta < -radians(180):
            theta = theta + radians(360)
        return theta