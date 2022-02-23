package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class Ducks {

    private LinearOpMode opMode;
    private RobotHardware hardware;

    private double power;
    private double pivot;

    public Ducks(LinearOpMode opMode, RobotHardware robotHardware) {
        this.opMode = opMode;
        this.hardware = robotHardware;
    }

    public void update() {
        hardware.getCRServo("duckSpin").setPower(power);
        hardware.getServo("duckPivot").setPosition(pivot);
    }

}
