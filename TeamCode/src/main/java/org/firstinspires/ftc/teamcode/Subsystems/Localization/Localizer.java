package org.firstinspires.ftc.teamcode.Subsystems.Localization;

public abstract class Localizer {

    public double x, y, heading;
    public double xVel, yVel, headingVel;
    public double xAcc, yAcc, headingAcc;

    public double loopTime, time;
    protected double lastX, lastY, lastHeadingRadians, headingRadians;

    public void initialize() {
        x = 0;
        y = 0;
        heading = 0;
        xVel = 0;
        yVel = 0;
        headingVel = 0;
        xAcc = 0;
        yAcc = 0;
        headingAcc = 0;
        time = 0;
        loopTime = 1;
    }
    public void startTracking(double initialX, double initialY, double initialHeading) {

        lastX = initialX;
        lastY = initialY;
        headingRadians = Math.toRadians(initialHeading);
        lastHeadingRadians = headingRadians;

    }

    public void update() {}

}
