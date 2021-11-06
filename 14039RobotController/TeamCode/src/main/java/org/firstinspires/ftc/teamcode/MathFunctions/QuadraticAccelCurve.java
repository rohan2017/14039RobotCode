package org.firstinspires.ftc.teamcode.MathFunctions;

public class QuadraticAccelCurve {

    private final double maxAccel = 0.1; // absolute
    private final double maxVel = 2; // absolute
    //ax^2 + bx + c
    private final double a = maxAccel/2;
    private double xOffset; // how much to shift function to the right
    private double accelPhaseEnd = maxVel/maxAccel;
    private double deccelPhaseEnd = -accelPhaseEnd;

    public QuadraticAccelCurve(double distance) {
        if(a*Math.pow(accelPhaseEnd, 2) > distance/2) {
            // no room for linear segment

        }else {

        }
    }

    public double getVelocity() {

    }

}
