package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Intake {

    private LinearOpMode opMode;
    private RobotHardware hardware;

    private double power;

    public Intake(LinearOpMode opMode, RobotHardware robothardware) {
        this.opMode = opMode;
        this.hardware = robothardware;
    }

    public void initialize() {
        hardware.getMotor("intake").setDirection(DcMotor.Direction.REVERSE);
    }

    public void setPower(double pow){
        power = pow;
    }

    public void update () {
        hardware.getMotor("intake").setPower(power);
    }


}
