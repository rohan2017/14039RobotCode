package org.firstinspires.ftc.teamcode.Subsystems.Movement;

import org.firstinspires.ftc.teamcode.Subsystems.Movement.Drivebases.Drivebase;

public abstract class Movement {

    private Drivebase drivebase;

    public Movement(Drivebase drivebase) {
        this.drivebase = drivebase;
    }

}
