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
        hardware.getMotor("intake").setDirection(DcMotor.Direction.REVERSE);
        hardware.getMotor("intake").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPower(double pow) {
        if(pow < 1 && pow > -1) power = pow;
    }

    public void update () {
        if(opMode.opModeIsActive()) {
            hardware.getMotor("intake").setPower(power);

            if((leftPos + rightPos) == 1) {
                hardware.getServo("leftFlipper").setPosition(leftPos);
                hardware.getServo("rightFlipper").setPosition(rightPos);
            }
        }
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
