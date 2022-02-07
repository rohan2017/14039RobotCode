package org.firstinspires.ftc.teamcode.MathFunctions;

public class PointEx {

    public double x, y, heading;
    public double speed;

    public PointEx(double x, double y, double heading){

        this.x = x;
        this.y = y;
        this.heading = heading;

    }

    public PointEx(double x, double y, double heading, double speed) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.speed = speed;
    }

    public String toString() {
        return "X:" + x + ", Y:" + y + ", H:" + heading;
    }

}
