package org.firstinspires.ftc.teamcode.Subsystems.Movement;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.MathFunctions.PointEx;
import org.firstinspires.ftc.teamcode.Controllers.TrapezoidalCurve;
import org.firstinspires.ftc.teamcode.Subsystems.Localization.Odometer;
import org.firstinspires.ftc.teamcode.Subsystems.Movement.Drivebases.DrivebaseHolonomic;

import static org.firstinspires.ftc.teamcode.MathFunctions.MyMath.*;
import java.util.ArrayList;

public class MovementHolonomic extends Movement {

    public double targetX, targetY, targetHeading;
    public double distanceThreshold = 2;
    public double headingThreshold = 2;

    private DrivebaseHolonomic drivebase;
    private Odometer odometer;
    private LinearOpMode opMode;

    private PID orient;
    private TrapezoidalCurve speedFinder;

    public MovementHolonomic (LinearOpMode opmode, DrivebaseHolonomic drivebase, Odometer odometer) {
        super(drivebase);
        this.drivebase = drivebase;
        this.odometer = odometer;
        this.opMode = opmode;
    }

    public void initialize() {
        targetX = 0;
        targetY = 0;
        targetHeading = 0;

        orient = new PID(0.3,0,0.2,0,0.4,0);
        speedFinder = new TrapezoidalCurve(10, 0.8);
        state = "idle";
    }

    public void update() {
        if(opMode.opModeIsActive()) {
            if(!state.equals("idle")) {
                // State determination
                if (distance(odometer.x, odometer.y, targetX, targetY) < distanceThreshold && (Math.abs(odometer.heading - targetHeading) < headingThreshold)) {
                    state = "converged";
                }else {
                    state = "transient";
                }
                // Actions
                if (state.equals("transient")) {

                    double xDist, yDist, distance, heading;
                    double targSpeed, scale;
                    double targVX, targVY, hCorrect;

                    xDist = targetX - odometer.x;
                    yDist = targetY - odometer.y;
                    distance = Math.hypot(xDist, yDist);
                    heading = odometer.heading;

                    targSpeed = Math.abs(speedFinder.correction);
                    scale = targSpeed / distance;

                    targVX = xDist * scale;
                    targVY = yDist * scale;
                    // Verified ^

                    speedFinder.update(0, distance);
                    orient.update(targetHeading, heading);

                    hCorrect = orient.correction;

                    setGlobalVelocity(targVX, targVY, hCorrect);

                    drivebase.update();
                } else if (state.equals("converged")) {
                    drivebase.freeze();
                }
                odometer.update();
            }else {
                drivebase.freeze();
            }
        }
    }

    public void setGlobalVelocity(double xVel, double yVel, double hVel) { // Verified
        if(opMode.opModeIsActive()) {
            double h = odometer.heading;
            double xRelVel = cosine(-h) * xVel - sine(-h) * yVel;
            double yRelVel = sine(-h) * xVel + cosine(-h) * yVel;
            drivebase.setRelativeVelocity(xRelVel, yRelVel, hVel);
        }
    }

    public void setTargets(double X, double Y, double Heading) {
        this.targetX = X;
        this.targetY = Y;
        this.targetHeading = Heading;
        updateControllers();
        state = "transient";
    }

    public void setTarget(PointEx target) {
        setTargets(target.x, target.y, target.heading);
    }

    public void setTargetPath(ArrayList<PointEx> path) {}

    private void updateControllers() {
        speedFinder = new TrapezoidalCurve(distance(targetX, targetY, odometer.x, odometer.y), 0.8);
    }

    public double getDistance() {
        return distance(targetX, targetY, odometer.x, odometer.y);
    }
    public double getSpeed() {
        return speedFinder.correction;
    }
}
