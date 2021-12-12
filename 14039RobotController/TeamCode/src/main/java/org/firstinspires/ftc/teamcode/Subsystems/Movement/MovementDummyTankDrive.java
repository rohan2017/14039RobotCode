package org.firstinspires.ftc.teamcode.Subsystems.Movement;

import static org.firstinspires.ftc.teamcode.MathFunctions.MyMath.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.Subsystems.Localization.Odometer;
import org.firstinspires.ftc.teamcode.Subsystems.Movement.Drivebases.DummyTankDrive;

public class MovementDummyTankDrive extends Movement {

    private DummyTankDrive drivebase;

    private double targetHeading;
    private double targetDistance;

    private double currentDistance;
    private double currentHeading;

    private PID orient;
    private PID lateral;

    public MovementDummyTankDrive(LinearOpMode opMode, DummyTankDrive drivebase, Odometer odometer) {
        super(opMode, odometer);
        this.drivebase = drivebase;

        orient = new PID(0.1,0,0.1,0,0.4,0);
        lateral = new PID(0.1,0,0.2,0,0.8,0);
    }

    public void initialize() {
        targetDistance = 0;
        targetHeading = 0;
        state = "idle";
    }

    public void update() {
        if(opMode.opModeIsActive()) {
            if(!state.equals("idle")) {
                // State determination
                currentDistance = (drivebase.getLeftEncoder() + drivebase.getRightEncoder())/2;
                currentHeading = odometer.heading;
                if (distance(targetHeading, targetDistance, currentHeading, currentDistance) < 30) {
                    state = "converged";
                }else {
                    state = "transient";
                }
                // Actions
                if (state.equals("transient")) {

                    orient.update(targetHeading, currentHeading);
                    lateral.update(targetDistance, currentDistance);
                    drivebase.setPowers(lateral.correction - orient.correction, lateral.correction + orient.correction);

                    drivebase.update();
                } else if (state.equals("converged")) {
                    drivebase.freeze();
                }
            }else {
                drivebase.freeze();
            }
            odometer.update();
        }
    }

    public void setTargets(double targetDistance, double targetHeading) {
        this.targetHeading = targetHeading;
        this.targetDistance = targetDistance;
        drivebase.resetDriveEncoders();
        state = "transient";
    }

    public double getDistToTarget(){
        currentDistance = (drivebase.getLeftEncoder() + drivebase.getRightEncoder())/2;
        currentHeading = odometer.heading;
        return distance(targetHeading, targetDistance, currentHeading, currentDistance);
    }

    public double getLateralCorrect(){
        return lateral.correction;
    }
    public double getHeadingCorrect(){
        return orient.correction;
    }

}
