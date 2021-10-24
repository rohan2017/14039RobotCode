package org.firstinspires.ftc.teamcode.Subsystems.Localization;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Odometer2WIMU extends Odometer{

    private LinearOpMode opMode;
    private RobotHardware hardware;

    /* Top-Down View of The Bottom of a Robot
    /===================\
    |                   |
    |  A------X-----O   |   A represents the vertical dead-wheel, and can be placed anywhere
    |     ^         |   |
    |     H         |   |   B represents the horizontal dead-wheel, and can be placed anywhere
    |           V-> |   |
    |               B   |
    |                   |
    \===================/
    The above diagram is a top-down view of the bottom of the robot, meaning that when you are
    looking at the robot from the top, A is to the left, O is to the right, and B is to the back.
    X denotes the center of the robot
    H denotes the horizontalOffset variable, or distance from A to X.
    V denotes the verticalOffset variable, or the distance from O to B. Does not matter very much
    Odometer measurements can be in whatever units you want, as long as you use the same units for every constant
    */

    private final double horizontalOffset = 3.25;
    private final double verticalOffset = 17.75;

    // These variables allow you to set the direction of the encoders regardless of any reversing going on elsewhere
    private double verticalDirection = -1;
    private double horizontalDirection = 1;

    // Encoder Variables
    public  double vertical, horizontal;
    private double lastVertical, lastHorizontal;
    private double ticksToDistance;

    private double lastHeadingImu;
    private double lastHeadingVel;

    private double[] totalPositionChange = {0,0};

    public Odometer2WIMU(LinearOpMode opMode, RobotHardware robothardware) {

        this.opMode = opMode;
        this.hardware = robothardware;

    }

    @Override
    public void initialize() {
        super.initialize();
        lastVertical = 0;
        lastHorizontal = 0;
        lastHeadingVel = 0;
        ticksToDistance = wheelRadius*2*Math.PI/ticksPerRevolution*gearRatio;

    }

    @Override
    public void update() {

        if(opMode.opModeIsActive()) {
            time = hardware.getTime();

            // HERE IS WHERE TO CHANGE ENCODER OBJECTS
            vertical = hardware.getMotor("verticalEncoder").getCurrentPosition() * ticksToDistance * verticalDirection;
            horizontal = hardware.getMotor("horizontalEncoder").getCurrentPosition() * ticksToDistance * horizontalDirection;

            if(firstloop) {
                lastVertical = vertical;
                lastHorizontal = horizontal;
                lastHeadingImu = hardware.getImuHeading("hub1");
                firstloop = false;
            }

            double verticalChange = vertical - lastVertical;
            double horizontalChange = horizontal - lastHorizontal;

            double headingImu = hardware.getImuHeading("hub1");

            // Math Variables
            double headingChange = headingImu - lastHeadingImu;

            if (headingChange < -Math.PI){ // For example 355 to 2 degrees
                headingChange = 2*Math.PI + headingChange;
            }else if (headingChange > Math.PI) { // For example 2 to 355 degrees
                headingChange = -2*Math.PI + headingChange;
            }

            headingRadians += headingChange;

            totalPositionChange = findPoseTransform(verticalChange, horizontalChange, headingChange);

            x = lastX + totalPositionChange[0];
            y = lastY + totalPositionChange[1];

            lastX = x;
            lastY = y;
            lastHeadingRadians = headingRadians;
            lastHeadingImu = headingImu;

            lastVertical = vertical;
            lastHorizontal = horizontal;

            heading = Math.toDegrees(headingRadians);

            loopTime = hardware.getTime() - time;
            calculateVelocity(headingChange);
            calculateAcceleration();
        }
    }

    private double[] findPoseTransform(double verticalChange, double horizontalChange, double headingChange) {
        // Calculating the position-change-vector from vertical encoder
        double verticalAdjust = horizontalOffset * headingChange;
        double verticalExtra = verticalChange - verticalAdjust;

        double[] positionChangeVertical = new double[2];
        positionChangeVertical[1] = Math.cos(headingChange) * verticalExtra;
        positionChangeVertical[0] = Math.sin(headingChange) * verticalExtra;

        //Calculating the position-change-vector from horizontal encoder
        double horizontalAdjust = verticalOffset * headingChange;
        double horizontalExtra = horizontalChange - horizontalAdjust;

        double[] positionChangeHorizontal = new double[2];
        positionChangeHorizontal[0] = Math.cos(headingChange) * horizontalExtra;
        positionChangeHorizontal[1] = Math.sin(headingChange) * horizontalExtra;

        //Add the two vectors together
        double[] totalRelativeMovement = new double[2];
        totalRelativeMovement[0] = positionChangeVertical[0] + positionChangeHorizontal[0];
        totalRelativeMovement[1] = positionChangeVertical[1] + positionChangeHorizontal[1];

        //Rotate the vector
        double[] totalPositionChange = new double[2];
        totalPositionChange[0] = totalRelativeMovement[0] * Math.cos(lastHeadingRadians) - totalRelativeMovement[1] * Math.sin(lastHeadingRadians);
        totalPositionChange[1] = totalRelativeMovement[0] * Math.sin(lastHeadingRadians) + totalRelativeMovement[1] * Math.cos(lastHeadingRadians);

        return totalPositionChange.clone();
    }
    /*
    private void calculateVelocity(double xChange, double yChange, double headingChange) {
        xVel = totalPositionChange[0]/loopTime*1000;
        yVel = totalPositionChange[1]/loopTime*1000;
        headingVel = headingChange/loopTime*1000;
    }
    */
    private void calculateVelocity(double headingChange) { // Using encoder velocities
        double verticalVel = hardware.getMotor("verticalEncoder").getVelocity() * ticksToDistance * verticalDirection;
        double horizontalVel = hardware.getMotor("verticalEncoder").getVelocity() * ticksToDistance * verticalDirection;
        headingVel = headingChange/loopTime*1000;
        double[] vel = findPoseTransform(verticalVel, horizontalVel, headingVel);
        xVel = vel[0];
        yVel = vel[1];
    }

    private void calculateAcceleration() {
        double[] acc = hardware.getImuAcc();
        xAcc = acc[0];
        yAcc = acc[1];
        headingAcc = (headingVel - lastHeadingVel)/loopTime*1000;
        lastHeadingVel = headingVel;
    }

    // Utility Methods
    public void setEncoderDirections(double verticalDirection, double horizontalDirection){
        this.verticalDirection = verticalDirection;
        this.horizontalDirection = horizontalDirection;
    }
}
