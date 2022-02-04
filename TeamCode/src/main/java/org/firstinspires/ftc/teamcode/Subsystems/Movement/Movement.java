package org.firstinspires.ftc.teamcode.Subsystems.Movement;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Localization.Odometer;
import org.firstinspires.ftc.teamcode.Subsystems.Movement.Drivebases.Drivebase;
import org.firstinspires.ftc.teamcode.Subsystems.Movement.Drivebases.DrivebaseHolonomic;
import org.firstinspires.ftc.teamcode.Subsystems.State;

public abstract class Movement {

    public State state;
    protected Odometer odometer;
    protected LinearOpMode opMode;

    public Movement(LinearOpMode opmode, Odometer odometer) {
        this.odometer = odometer;
        this.opMode = opmode;
    }
    public void initialize(){}
    public void update(){}

}
