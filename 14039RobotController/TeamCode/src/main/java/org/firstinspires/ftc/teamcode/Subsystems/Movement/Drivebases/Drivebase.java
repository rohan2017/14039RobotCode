package org.firstinspires.ftc.teamcode.Subsystems.Movement.Drivebases;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public abstract class Drivebase {

    protected LinearOpMode opMode;
    protected RobotHardware hardware;

    public Drivebase(LinearOpMode opMode, RobotHardware hardware) {
        this.opMode = opMode;
        this.hardware = hardware;
    }

    public void initialize() {}
    public void update() {}
    public void freeze() {}

}
