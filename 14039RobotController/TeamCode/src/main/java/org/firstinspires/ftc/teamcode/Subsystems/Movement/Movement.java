package org.firstinspires.ftc.teamcode.Subsystems.Movement;

import org.firstinspires.ftc.teamcode.Subsystems.Movement.Drivebases.Drivebase;

public abstract class Movement {

    private Drivebase drivebase;
    public String state;

    public Movement(Drivebase drivebase) {
        this.drivebase = drivebase;
    }

    public void initialize(){}
    public void update(){}

}
