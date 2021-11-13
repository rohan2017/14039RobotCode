package org.firstinspires.ftc.teamcode.Controllers;

public class TrapezoidalCurve extends Controller {

    private double maxVel, minVelAccel, minVelDeccel;
    private double distance;
    private double accelRange, deccelRange;
    private double aSlope, dSlope;

    public TrapezoidalCurve(double maxVel, double minVelAccel, double minVelDeccel, double accelRange, double deccelRange, double distance) {
        this.distance = distance;

        this.minVelAccel = minVelAccel;
        this.minVelDeccel = minVelDeccel;

        this.accelRange = accelRange;
        this.deccelRange = deccelRange;
        if(distance > (accelRange + deccelRange)) { // If there is enough distance to fully accelerate
            this.maxVel = maxVel;
        }else { // If not, scale down the curve
            this.accelRange = (accelRange + deccelRange)/distance * accelRange;
            this.deccelRange = (accelRange + deccelRange)/distance * deccelRange;
            this.maxVel = maxVel * (accelRange + deccelRange)/distance;
        } //                                                              ___
        aSlope = (maxVel-minVelAccel)/(accelRange-0); //Rise over run ___/                ___
        dSlope = (maxVel-minVelDeccel)/((distance-deccelRange)-distance); //Rise over run    \___

        correction = 0;
    }

    public void setDistance(double distance) {
        this.distance = distance;
        if(distance > (accelRange + deccelRange)) { // If there is enough distance to fully accelerate
            // No change
        }else { // If not, scale down the curve
            accelRange = (accelRange + deccelRange)/distance * accelRange;
            deccelRange = (accelRange + deccelRange)/distance * deccelRange;
        }
    }

    public double getDistance() {
        return distance;
    }

    public void update(double setPoint, double currentDistance) {
        double input = distance - currentDistance;
        if(input < accelRange && input >= 0){ // If inside the acceleration range
            correction = aSlope * input + minVelAccel;
        }else if(input > accelRange && input < (distance-deccelRange)){ // If inside the maxVel range
            correction = maxVel;
        }else if(input > (distance-deccelRange) && input < distance){ // If inside the de-accelerate range
            correction = dSlope * (input-distance+deccelRange) + minVelDeccel;
        }else {
            correction = 0;
        }
    }
}
