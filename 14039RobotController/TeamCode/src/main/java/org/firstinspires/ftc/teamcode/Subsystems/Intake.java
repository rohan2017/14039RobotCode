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
    private double leftExtendPos, rightExtendPos;

    public boolean hasBlock = false;
    public double intensity;
    public double filteredIntensity = 0;

    public Intake(LinearOpMode opMode, RobotHardware robothardware) {
        this.opMode = opMode;
        this.hardware = robothardware;
    }

    public void initialize() {

        hardware.getMotor("intake").setDirection(DcMotor.Direction.REVERSE);
        hardware.getMotor("intake").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.getMotor("intake").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        power = 0;
        filteredIntensity = 0;
        flipHold();
        retract();
        update();
    }

    public void setPower(double pow) {
        if(pow <= 1 && pow >= -1) power = pow;
    }

    public void update() {
        if(opMode.opModeIsActive()) {
            intensity = hardware.getRangeSensor("intake_range").rawOptical();
            if(intensity >= 0 && intensity < 200) {
                filteredIntensity += 0.9*(intensity - filteredIntensity);
            }
            hasBlock = filteredIntensity > 18;

            hardware.getMotor("intake").setPower(power);

            if((leftPos + rightPos) == 1) {
                hardware.getServo("leftFlipper").setPosition(leftPos);
                hardware.getServo("rightFlipper").setPosition(rightPos);
            }

            if((leftExtendPos + rightExtendPos) == 1) {
                hardware.getServo("leftExtend").setPosition(leftExtendPos);
                hardware.getServo("rightExtend").setPosition(rightExtendPos);
            }
        }
    }

    public void flipHold(){
        leftPos = 0.22;
        rightPos = 0.78;
    }
    public void flipUp() {
        leftPos = 0.75;
        rightPos = 0.25;
    }

    public void flipDown() {
        leftPos = 0.1;
        rightPos = 0.9;
    }

    public void retract() {
        rightExtendPos = 1;
        leftExtendPos = 0;
    }

    public void setFlipPosition(double pos) {
        if (pos < 0) pos = 0;
        if (pos > 0.62) pos = 0.62;
        rightPos = 0.97 - pos;
        leftPos = 0.03 + pos;
    }

    public void setExtendPosition(double pos) {
        if(pos < 0) pos = 0;
        if(pos > 0.33) pos = 0.33;
        rightExtendPos = 1-pos;
        leftExtendPos = 0+pos;
    }

}
