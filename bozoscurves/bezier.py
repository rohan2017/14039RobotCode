import math


class BezierCubic:

    point0start = (0, 0)
    point1corner = (0, 0)
    point2corner = (0, 0)
    point3end = (0, 0)

    def __init__(self, p0t, p1c, p2c, p3t):
        self.point0start = p0t
        self.point1corner = p1c
        self.point2corner = p2c
        self.point3end = p3t

    def get_point(self, t):
        tcubed = math.pow(t, 3)
        tsquared = math.pow(t, 2)
        p0factor = ((3 * tsquared) - tcubed - (3 * t) + 1)
        p1factor = ((3 * tcubed) - (6 * tsquared) + 3 * t)
        p2factor = ((3 * tsquared) - (3 * tcubed))
        x = self.point0start[0] * p0factor
        x += self.point1corner[0] * p1factor
        x += self.point2corner[0] * p2factor
        x += self.point3end[0] * tcubed

        y = self.point0start[1] * p0factor
        y += self.point1corner[1] * p1factor
        y += self.point2corner[1] * p2factor
        y += self.point3end[1] * tcubed
        return x, y

    def get_point_d1(self, t):
        tsquared = math.pow(t, 2)
        p0factor = ((6 * t) - (3 * tsquared) - 3)
        p1factor = ((9 * tsquared) - (12 * t) + 3)
        p2factor = ((6 * t) - (9 * tsquared))
        x = self.point0start[0] * p0factor
        x += self.point1corner[0] * p1factor
        x += self.point2corner[0] * p2factor
        x += self.point3end[0] * (3 * tsquared)

        y = self.point0start[1] * p0factor
        y += self.point1corner[1] * p1factor
        y += self.point2corner[1] * p2factor
        y += self.point3end[1] * (3 * tsquared)
        return x, y

    def get_point_d2(self, t):
        x = self.point0start[0] * (6 - (6 * t))
        x += self.point1corner[0] * ((18 * t) - 12)
        x += self.point2corner[0] * (6 - (18 * t))
        x += self.point3end[0] * (6 * t)

        y = self.point0start[1] * (6 - (6 * t))
        y += self.point1corner[1] * ((18 * t) - 12)
        y += self.point2corner[1] * (6 - (18 * t))
        y += self.point3end[1] * (6 * t)
        return x, y

    def get_point_d3(self, t):
        x = (self.point1corner[0] * 18) - (self.point0start[0] * 6) + (self.point3end[0] * 6) - (self.point2corner[0] * 18);
        y = (self.point1corner[1] * 18) - (self.point0start[1] * 6) + (self.point3end[1] * 6) - (self.point2corner[1] * 18);
        return x, y
