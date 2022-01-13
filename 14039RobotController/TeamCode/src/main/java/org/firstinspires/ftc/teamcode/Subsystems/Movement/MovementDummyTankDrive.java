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
    private PID longitudinal;

    public MovementDummyTankDrive(LinearOpMode opMode, DummyTankDrive drivetrain, Odometer odometer) {
        super(opMode, odometer);
        this.drivebase = drivetrain;
        orient = new PID(0.03,0,0.05,0,0.2,0);
        longitudinal = new PID(0.15,0,0.15,0,0.5,0);
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
                if (distance(targetHeading, targetDistance, currentHeading, currentDistance) < 5) {
                    state = "converged";
                }else {
                    state = "transient";
                }
                // Actions

                if (state.equals("transient")) {
                    orient.update(targetHeading, currentHeading);
                    longitudinal.update(targetDistance, currentDistance);
                    drivebase.setPowers(longitudinal.correction - orient.correction, longitudinal.correction + orient.correction);                    drivebase.update();
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
        //drivebase.resetDriveEncoders();
        state = "transient";
    }

    public double getDistToTarget(){
        currentDistance = (drivebase.getLeftEncoder() + drivebase.getRightEncoder())/2;
        currentHeading = odometer.heading;
        return distance(targetHeading, targetDistance, currentHeading, currentDistance);
    }

    public double getLongitudinalCorrect(){
        return longitudinal.correction;
    }
    public double getHeadingCorrect(){
        return orient.correction;
    }
    public double getHeading() {return odometer.heading;}

}
