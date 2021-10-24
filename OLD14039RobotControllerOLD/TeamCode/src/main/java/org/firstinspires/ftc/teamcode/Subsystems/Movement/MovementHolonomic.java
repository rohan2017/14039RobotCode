package org.firstinspires.ftc.teamcode.Subsystems.Movement;

import org.firstinspires.ftc.teamcode.Subsystems.Movement.Drivebases.DrivebaseHolonomic;

public class MovementHolonomic extends Movement {

    public double targetX, targetY, targetHeading;

    public MovementHolonomic (DrivebaseHolonomic drivebase) {
        super(drivebase);
    }

    public void update() {

    }
}
