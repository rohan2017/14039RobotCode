package org.firstinspires.ftc.teamcode.Controllers;

public class SCurve extends Controller {

    private double distance;
    private double accelerationSlope = 5;
    private double decelerationSlope = -0.8;
    private double minVel = 0.05;
    private double maxVel = 1;

    private double t, d, c;

    public SCurve(double distance) {
        this.distance = distance;
        t = Math.log(maxVel/minVel - 1) / maxVel;
        d = t/decelerationSlope + distance;
        c = t/accelerationSlope;
        correction = 0;
    }

    public void update(double setPoint, double currentDistance) {
        double input = distance - currentDistance;
        double accelcorrect = maxVel / (1 + Math.pow(Math.E, (accelerationSlope * maxVel * (c - input))));
        double deccelcorrect = maxVel / (1 + Math.pow(Math.E, (decelerationSlope * maxVel * (d - input))));

        if(accelcorrect <= deccelcorrect) {
            correction = accelcorrect;
        }else {
            correction = deccelcorrect;
        }
    }
}
