package org.firstinspires.ftc.teamcode.MathFunctions;

public class BezierCurve { // A Cubic Bezier

    private PointEx point0start;
    private PointEx point1corner;
    private PointEx point2corner;
    private PointEx point3end;

    public BezierCurve(PointEx p0t, PointEx p1c, PointEx p2c, PointEx p3t) {
        this.point0start = p0t;
        this.point1corner = p1c;
        this.point2corner = p2c;
        this.point3end = p3t;
    }

    public PointEx getPoint(double t) { // Displacement
        double tcubed = Math.pow(t, 3);
        double tsquared = Math.pow(t, 2);
        double p0factor = ((3*tsquared) - (tcubed) - (3*t) + 1);
        double p1factor = ((3*tcubed) - (6*tsquared) + 3*t);
        double p2factor = ((3*tsquared) - (3*tcubed));
        double x = point0start.x * p0factor;
        x += point1corner.x * p1factor;
        x += point2corner.x * p2factor;
        x += point3end.x * (tcubed);
        double y = point0start.y * p0factor;
        y += point1corner.y * p1factor;
        y += point2corner.y * p2factor;
        y += point3end.y * (tcubed);
        return new PointEx(x, y, 0);
    }

    public PointEx getPointDerivative(double t) { // Velocity
        double tsquared = Math.pow(t, 2);
        double p0factor = ((6*t) - (3*tsquared) - 3);
        double p1factor = ((9*tsquared) - (12*t) + 3);
        double p2factor = ((6*t) - (9*tsquared));
        double x = point0start.x * p0factor;
        x += point1corner.x * p1factor;
        x += point2corner.x * p2factor;
        x += point3end.x * (3 * tsquared);
        double y = point0start.y * p0factor;
        y += point1corner.y * p1factor;
        y += point2corner.y * p2factor;
        y += point3end.y * (3 * tsquared);
        return new PointEx(x, y, 0);
    }

    public PointEx getPointDerivative2(double t) { // Acceleration
        double x = point0start.x * (6 - (6*t));
        x += point1corner.x * ((18*t) - 12);
        x += point2corner.x * (6 - (18*t));
        x += point3end.x * (6 * t);
        double y = point0start.y * (6 - (6*t));
        y += point1corner.y * ((18*t) - 12);
        y += point2corner.y * (6 - (18*t));
        y += point3end.y * (6 * t);
        return new PointEx(x, y, 0);
    }

    public PointEx getPointDerivative3() { // Jerk
        double x =  (point1corner.x * 18) - (point0start.x * 6) + (point3end.x * 6) - (point2corner.x * 18);
        double y =  (point1corner.y * 18) - (point0start.y * 6) + (point3end.y * 6) - (point2corner.y * 18);
        return new PointEx(x, y, 0);
    }

    public double getT(PointEx point) { // Assuming it's a point on the curve
        return 0.0;
    }

}
