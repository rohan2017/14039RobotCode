package org.firstinspires.ftc.teamcode.MathFunctions;

public class TrapezoidalCurve extends VelocityCurve {

    // Robot intrinsic constants
    private double maxVel;
    private final double minVelAccel = 0.02;
    private final double minVelDeccel = 0;
    private final double aSlope = 1;
    private final double dSlope = -0.04;

    // Calculation Variables
    private double intersection;
    private double distance, time;
    private double accelPhase = 0; // length of acceleration time
    private double holdPhase = 0; // duration of maintain top speed
    private double decelPhase = 0; // length of deceleration time

    public TrapezoidalCurve(double distanceTotal, double maxVel) {
        this.distance = distanceTotal;
        this.maxVel = maxVel;

        accelPhase = maxVel/aSlope;
        decelPhase = maxVel/(-dSlope);
        double rampDistance =(accelPhase+decelPhase)*maxVel;

        if(distance > rampDistance) { // if there is time to reach maxVel
            holdPhase = (distance-rampDistance)/maxVel;
            time = holdPhase + accelPhase + decelPhase;
            intersection = (dSlope * time)/(dSlope-aSlope);
        }else {
            double area = aSlope*dSlope / (dSlope-aSlope) / 2;
            time = Math.sqrt(distance/area);
            intersection = (dSlope * time)/(dSlope-aSlope);
            accelPhase = intersection;
            decelPhase = time-intersection;
            this.maxVel = aSlope*intersection;
        }
    }

    public void setDistance(double distanceTotal) {
        this.distance = distanceTotal;
        this.maxVel = maxVel;

        accelPhase = maxVel/aSlope;
        decelPhase = maxVel/(-dSlope);
        double rampDistance =(accelPhase+decelPhase)*maxVel;

        if(distance > rampDistance) { // if there is time to reach maxVel
            holdPhase = (distance-rampDistance)/maxVel;
            time = holdPhase + accelPhase + decelPhase;
            intersection = (dSlope * time)/(dSlope-aSlope);
        }else {
            double area = aSlope*dSlope / (dSlope-aSlope) / 2;
            time = Math.sqrt(distance/area);
            intersection = (dSlope * time)/(dSlope-aSlope);
            accelPhase = intersection;
            decelPhase = time-intersection;
            this.maxVel = aSlope*intersection;
        }
    }

    private double getVelocity(double currentTime) {
        double velocity = 0;
        if(currentTime <= intersection) {
            velocity = aSlope * currentTime;
            if(velocity < minVelAccel) {
                velocity = minVelAccel;
            }
        }else if(currentTime > intersection) {
            velocity = dSlope*currentTime - dSlope*time;
            if(velocity < minVelDeccel) {
                velocity = minVelDeccel;
            }
        }
        if(velocity > maxVel) {
            velocity = maxVel;
        }
        return velocity;
    }

    private double getTime(double distanceTravelled) {
        double accelArea = accelPhase*maxVel/2;
        double decelArea = decelPhase*maxVel/2;
        double holdArea = holdPhase*maxVel;
        double remainingDistance;
        if(distanceTravelled <= accelArea) { // time is in accel phase
            return Math.sqrt(2*distanceTravelled/aSlope);
        }else if(distanceTravelled < distance-decelArea) { // time is in hold phase
            remainingDistance = distanceTravelled - accelArea;
            return accelPhase + remainingDistance/maxVel;
        }else { // time is in decel phase
            remainingDistance = distanceTravelled - (accelArea+holdArea);
            return accelPhase + holdPhase + Math.sqrt(-2*remainingDistance/dSlope);
        }
    }

    public double getTargetVelocity(double distanceTravelled) {
        return getVelocity(getTime(distanceTravelled));
    }

    public double getTargetVelocity(double distanceRemaining, int i) {
        return getVelocity(getTime(distance - distanceRemaining));
    }
}
