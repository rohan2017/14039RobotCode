package org.firstinspires.ftc.teamcode.Subsystems.Movement.Drivebases;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class DrivebaseHolonomic extends Drivebase {

    public DrivebaseHolonomic(LinearOpMode opMode, RobotHardware hardware) {
        super(opMode, hardware);
    }

    @Override
    public void initialize() {

    }
    public void update() {}
    public void freeze() {}

    public void setRelativeVelocity(double velX , double velY, double velHeading) {}

}