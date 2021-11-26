package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Outtake {

    private LinearOpMode opMode;
    private RobotHardware hardware;

    private double powerLift, powerArm;
    private String doorState;
    public double rotatorPosition;

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
