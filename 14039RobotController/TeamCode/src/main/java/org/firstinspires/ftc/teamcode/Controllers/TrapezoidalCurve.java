package org.firstinspires.ftc.teamcode.Controllers;

public class TrapezoidalCurve extends Controller {

    // Robot intrinsic constants
    private double maxVel;
    private final double minVelAccel = 0.02;
    private final double minVelDeccel = 0;
    private double aSlope = 1;
    private double dSlope = -0.04;

    // Calculation Variables
    private double intersection;
    private double distance;

    public TrapezoidalCurve(double distance, double maxVel) {
        this.distance = distance;
        this.maxVel = maxVel;
        intersection = -(dSlope * distance)/(aSlope-dSlope);
        correction = 0;
    }

    public void update(double setPoint, double currentDistance) {
        double input = distance - currentDistance;
        if(input <= intersection) {
            correction = aSlope * input;
            if(correction < minVelAccel) {
                correction = minVelAccel;
            }
        }else if(input > intersection) {
            correction = dSlope * (-currentDistance);
            if(correction < minVelDeccel) {
                correction = minVelDeccel;
            }
        }
        if(correction > maxVel) {
            correction = maxVel;
        }
    }
}
