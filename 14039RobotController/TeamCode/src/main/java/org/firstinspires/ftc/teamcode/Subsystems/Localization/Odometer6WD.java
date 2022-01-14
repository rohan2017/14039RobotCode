package org.firstinspires.ftc.teamcode.Subsystems.Localization;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class Odometer6WD extends Odometer {

    private LinearOpMode opMode;
    private RobotHardware hardware;

    // Horizontal distance between left and right wheel lines
    private final double wheelBase = 24.35;
    private final double driveWheelRadius = 5.08;
    private final double driveMotorTPR = 537.6;
    private final double driveGearRatio = 1; // Wheel rpm / Motor rpm

    // Encoder Variables
    public  double right, left;
    private double lastRight, lastLeft;
    private double ticksToDistance;

    private double lastHeadingImu;

    public Odometer6WD(LinearOpMode opMode, RobotHardware robothardware) {
        this.opMode = opMode;
        this.hardware = robothardware;
    }

    @Override
    public void initialize() {
        super.initialize();
        lastRight = 0;
        lastLeft = 0;
        ticksToDistance = driveWheelRadius*2*Math.PI/driveMotorTPR*driveGearRatio;
        startTracking(0,0,0);
    }

    @Override
    public void update() {

        if(opMode.opModeIsActive()) {
            time = hardware.getTime();
            right = (hardware.getMotor("driveFrontRight").getCurrentPosition() + hardware.getMotor("driveBackRight").getCurrentPosition())/2.0 * ticksToDistance;
            left = (hardware.getMotor("driveFrontLeft").getCurrentPosition() + hardware.getMotor("driveBackLeft").getCurrentPosition())/2.0 * ticksToDistance;

            if(firstloop) {
                lastLeft = left;
                lastRight = right;
                lastHeadingImu = hardware.getImuHeading("hub1");
                firstloop = false;
            }

            double rightChange = right - lastRight;
            double leftChange = left - lastLeft;

            double headingImu = hardware.getImuHeading("hub1");

            // Math Variables
            double headingChange = headingImu - lastHeadingImu;
            //headingChange *= 1.013; // Weird IMU calibration issue

            if (headingChange < -Math.PI){ // For example 355 to 2 degrees
                headingChange = 2*Math.PI + headingChange;
            }else if (headingChange > Math.PI) { // For example 2 to 355 degrees
                headingChange = -2*Math.PI + headingChange;
            }

            headingRadians += headingChange;

            // SLIP CORRECTION
            double expectedDifference = headingChange*wheelBase;
            double correction = (rightChange-leftChange) - expectedDifference; // actual - expected
            double rightChangeBiased = rightChange - correction*0.3;
            double leftChangeBiased = leftChange + correction*0.3;

            double[] totalPositionChange = new double[2];
            double[] relativeMovement = new double[2];

            double centerArc = (leftChangeBiased + rightChangeBiased) / 2;

            double turnRadius;
            if(Math.abs(headingChange) < 0.00001) { // Robot has gone straight/not moved

                relativeMovement[0] = 0;
                relativeMovement[1] = centerArc;

            }else if(Math.abs(rightChangeBiased) < Math.abs(leftChangeBiased)){ // Left encoder is on inside of the turn

                turnRadius = centerArc / headingChange; // Always positive

                relativeMovement[0] = turnRadius - Math.cos(headingChange) * turnRadius;
                relativeMovement[1] = Math.sin(headingChange) * turnRadius;

            }else{ //Right encoder is on inside of the turn
                turnRadius = centerArc /-headingChange; // Always positive

                relativeMovement[0] = turnRadius - Math.cos(-headingChange) * turnRadius;
                relativeMovement[1] = Math.sin(-headingChange) * turnRadius;
            }

            totalPositionChange[0] = relativeMovement[0] * Math.cos(lastHeadingRadians) - relativeMovement[1] * Math.sin(lastHeadingRadians);
            totalPositionChange[1] = relativeMovement[0] * Math.sin(lastHeadingRadians) + relativeMovement[1] * Math.cos(lastHeadingRadians);

            x = lastX + totalPositionChange[0];
            y = lastY + totalPositionChange[1];

            lastX = x;
            lastY = y;
            lastHeadingRadians = headingRadians;
            lastHeadingImu = headingImu;

            lastLeft = left;
            lastRight = right;

            heading = Math.toDegrees(headingRadians);

            loopTime = hardware.getTime() - time;
        }
    }
}
