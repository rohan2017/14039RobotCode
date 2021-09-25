package org.firstinspires.ftc.teamcode.Localization;

public abstract class Localizer {

    public double x, y, heading, xVel, yVel, headingVel;
    public double loopTime;
    protected double lastX, lastY, lastHeadingRadians, headingRadians;

    public void initialize() {}
    public void startTracking(double initialX, double initialY, double initialHeading) {

        lastX = initialX;
        lastY = initialY;
        headingRadians = Math.toRadians(initialHeading);
        lastHeadingRadians = headingRadians;

    }

    public void update() {}

}
