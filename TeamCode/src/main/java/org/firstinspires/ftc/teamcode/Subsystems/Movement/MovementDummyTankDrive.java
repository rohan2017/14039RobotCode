package org.firstinspires.ftc.teamcode.Subsystems.Movement;

import static org.firstinspires.ftc.teamcode.MathFunctions.MyMath.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.Subsystems.Localization.Odometer;
import org.firstinspires.ftc.teamcode.Subsystems.Movement.Drivebases.DummyTankDrive;
import org.firstinspires.ftc.teamcode.Subsystems.State;

public class MovementDummyTankDrive extends Movement {

    private DummyTankDrive drivebase;

    private double targetHeading;
    private double targetDistance;

    public double currentDistance;
    public double currentHeading;

    private int mode = 0;

    private PID orient;
    private PID longitudinal;

    public MovementDummyTankDrive(LinearOpMode opMode, DummyTankDrive drivetrain, Odometer odometer) {
        super(opMode, odometer);
        this.drivebase = drivetrain;
        orient = new PID(0.01,0,0.05,0,0.2,0);
        longitudinal = new PID(0.15,0,0.1,0,0.5,0);
    }

    public void initialize() {
        targetDistance = 0;
        targetHeading = 0;
        state = State.IDLE;
    }

    public void update() {
        if(opMode.opModeIsActive()) {
            if(state != State.IDLE) {
                // State determination
                currentDistance = (drivebase.getLeftEncoder() + drivebase.getRightEncoder())/2;
                currentHeading = odometer.heading;

                if(mode == 0) {
                    if(Math.abs(targetDistance - currentDistance) < 5) {
                        state = State.CONVERGED;
                    }else {
                        state = State.TRANSIENT;
                    }
                }else {
                    if (Math.abs(targetHeading - currentHeading) < 5) {
                        state = State.CONVERGED;
                    }else {
                        state = State.TRANSIENT;
                    }
                }


                // Actions

                if (state == State.TRANSIENT) {
                    orient.update(targetHeading, currentHeading);
                    longitudinal.update(targetDistance, currentDistance);
                    if(mode == 0) {
                        drivebase.setPowers(longitudinal.correction - (0.5*orient.correction), longitudinal.correction + (0.5*orient.correction));
                    }else {
                        drivebase.setPowers(-orient.correction, orient.correction);
                    }
                    drivebase.update();
                } else if (state == State.CONVERGED) {
                    drivebase.freeze();
                }
            }else {
                drivebase.freeze();
            }
            odometer.update();
        }
    }

    public void setTargetDistance(double targetDistance) {
        this.targetDistance = targetDistance;
        this.targetHeading = currentHeading;
        state = State.TRANSIENT;
        mode = 0;
    }

    public void setTargetHeading(double targetHeading) {
        this.targetHeading = targetHeading;
        this.targetDistance = currentDistance;
        state = State.TRANSIENT;
        mode = 1;
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
