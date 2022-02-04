package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public abstract class Robot {
    public RobotHardware hardware;
    public Robot(LinearOpMode opMode) {}
    public void initialize() {}
    public void update() {} //updates all robot subsystems
}
