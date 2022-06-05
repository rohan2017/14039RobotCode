package org.firstinspires.ftc.teamcode.Controllers;

public class SCurve extends Controller {

    private double distance;
    private double time;
    private double accelerationSlope = 5;
    private double decelerationSlope = -0.8;
    private double minVel = 0.05;
    private double maxVel = 1;

    private double t, d, c;
    private double crossover;
    private double accelDistOffset, decelDistOffset;

    public SCurve(double distance) {
        this.distance = distance;

        t = Math.log(maxVel/minVel - 1) / maxVel;
        d = t/decelerationSlope + distance;
        c = t/accelerationSlope;
        crossover = (accelerationSlope*c - decelerationSlope*d) / (accelerationSlope-decelerationSlope);
        accelDistOffset = (Math.log(Math.pow(Math.E, accelerationSlope*maxVel*c) + 1)/accelerationSlope) - minVel;
        decelDistOffset = 0;
        decelDistOffset = evaluateDistance(crossover+0.0001) - evaluateDistance(crossover);
        correction = 0;
    }

    public void update(double setPoint, double currentDistance) {
        double input = distance - currentDistance;
        correction = evaluateVelocity(input);
    }

    public double evaluateVelocity(double input) {
        if(input <= crossover) {
            return maxVel / (1 + Math.pow(Math.E, (accelerationSlope * maxVel * (c - input))));
        }else if(input > crossover) {
            return maxVel / (1 + Math.pow(Math.E, (decelerationSlope * maxVel * (d - input))));
        }
        else{
            return  0;
        }
    }

    public double evaluateDistance(double input) {
        if(input <= crossover) {
            double tmp = Math.log(Math.pow(Math.E, accelerationSlope*maxVel*(c-input)) + 1);
            return (tmp/accelerationSlope) + (maxVel * input) - accelDistOffset;
        }else {
            double tmp = Math.log(Math.pow(Math.E, decelerationSlope*maxVel*(d-input)) + 1);
            return (tmp/decelerationSlope) + (maxVel * input) - decelDistOffset;
        }

    }
}
