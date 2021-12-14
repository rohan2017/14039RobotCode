package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class Intake {

    private LinearOpMode opMode;
    private RobotHardware hardware;

    private double power;
    private double leftPos, rightPos;

    public Intake(LinearOpMode opMode, RobotHardware robothardware) {
        this.opMode = opMode;
        this.hardware = robothardware;
    }

    public void initialize() {
        //hardware.getMotor("intake").setDirection(DcMotor.Direction.REVERSE);
    }

    public void setPower(double pow){
        power = pow;
    }

    public void update () {
        //hardware.getMotor("intake").setPower(power);
        hardware.getServo("leftFlipper").setPosition(leftPos);
        hardware.getServo("rightFlipper").setPosition(rightPos);
    }

    public void flipUp(){
        leftPos = 0;
        rightPos = 1;
    }

    public void flipDown(){
        leftPos = 0.62;
        rightPos = 0.38;
    }


}
