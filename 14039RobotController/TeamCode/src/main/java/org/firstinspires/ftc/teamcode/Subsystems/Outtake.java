package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.MathFunctions.MyMath;
import org.firstinspires.ftc.teamcode.MathFunctions.PointEx;

import java.util.ArrayList;

public class Outtake {

    private LinearOpMode opMode;
    private RobotHardware hardware;

    private double powerLift, powerArm;
    private String doorState;
    public double rotatorPosition;

    private final double armRad = 24.13;
    private final double dropDistance = 10;

    private double armAngle, rotatorAngle;

    public Outtake(LinearOpMode opMode, RobotHardware robothardware) {
        this.opMode = opMode;
        this.hardware = robothardware;
        rotatorPosition = 0.8;
        doorState = "closed";
        powerArm = 0;
        powerLift = 0;
    }

    public void initialize() {
        hardware.getMotor("lift").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.getMotor("lift").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.getMotor("lift").setDirection(DcMotor.Direction.REVERSE);
        hardware.getMotor("lift").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.getMotor("arm").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setLiftPower(double pow){
        powerLift = pow;
    }

    public void setArmPower(double pow){
        if(pow < 1 && pow > -0.8) {
            powerArm = pow;
        }
    }

    public void doorState(String state){
        doorState = state;
    }

    public double[] solveAngles(double targetX, double targetY) {
        ArrayList<PointEx> allPoints = new ArrayList<>();
        allPoints = MyMath.circleCircleIntersection(0,0,armRad,targetX,targetY,dropDistance);
        double armAngle, armAngle2, rotatorAngle, rotatorAngle2;
        if(allPoints.size() == 0) {
            //No intersections
            armAngle = 404;
            rotatorAngle = 404;
        }else if(allPoints.size() == 1) {
            // 1 point
            PointEx point = allPoints.get(0);
            armAngle = Math.toDegrees(Math.atan2(point.y, point.x));
            rotatorAngle = Math.toDegrees(Math.atan2(targetY-point.y, targetX-point.x));
        }else {
            // 2 points
            PointEx point = allPoints.get(0);
            armAngle = Math.toDegrees(Math.atan2(point.y, point.x));
            rotatorAngle = Math.toDegrees(Math.atan2(targetY-point.y, targetX-point.x));
            PointEx point2 = allPoints.get(0);
            armAngle2 = Math.toDegrees(Math.atan2(point2.y, point2.x));
            rotatorAngle2 = Math.toDegrees(Math.atan2(targetY-point2.y, targetX-point2.x));
        }

        double[] angles = {armAngle, rotatorAngle};
        return angles.clone();
    }

    public void setArmAngle(double angle) {
        // Straight out is 90 degrees
        /*
        0 ----.---- 180
              |
              |
              90
         */

    }

    public double getArmAngle(double angle) {
        return armAngle;
    }

    public void setRotatorAngle(double angle) {
        // Straight out is 90 degrees
        /*
        0 ----.---- 180
              |
              |
              90
         */
    }

    public double getRotatorAngle() {
        return rotatorAngle;
    }

    public void update () {
        if(hardware.getMotor("lift").getCurrentPosition() > 10) {
            hardware.getMotor("arm").setPower(powerArm);
            hardware.getServo("rotator").setPosition(rotatorPosition);

        }else {
            hardware.getMotor("arm").setPower(0);
            if(powerLift < 0) {
                powerLift = 0;
            }
            hardware.getServo("rotator").setPosition(0);
        }

        hardware.getMotor("lift").setPower(powerLift);

        if (doorState.equals("open")) {
            hardware.getServo("door").setPosition(0.21);
        }else {
            hardware.getServo("door").setPosition(0);
        }

    }

}
