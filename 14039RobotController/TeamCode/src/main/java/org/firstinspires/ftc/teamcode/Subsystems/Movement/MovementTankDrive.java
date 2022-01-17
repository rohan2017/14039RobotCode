package org.firstinspires.ftc.teamcode.Subsystems.Movement;
/*
import static org.firstinspires.ftc.teamcode.MathFunctions.MyMath.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.MathFunctions.PointEx;
import org.firstinspires.ftc.teamcode.Subsystems.Localization.Odometer;
import org.firstinspires.ftc.teamcode.Subsystems.Movement.Drivebases.DummyTankDrive;
import org.firstinspires.ftc.teamcode.Subsystems.State;

import java.util.ArrayList;

public class MovementTankDrive extends Movement {

    private DummyTankDrive drivebase;

    private PID orient;
    private PID longitudinal;

    public MovementTankDrive(LinearOpMode opMode, DummyTankDrive drivetrain, Odometer odometer) {
        super(opMode, odometer);
        this.drivebase = drivetrain;
        orient = new PID(0.03,0,0.05,0,0.2,0);
        longitudinal = new PID(0.15,0,0.15,0,0.5,0);
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
                if (distance(targetHeading, targetDistance, currentHeading, currentDistance) < 5) {
                    state = State.CONVERGED;
                }else {
                    state = State.TRANSIENT;
                }
                // Actions

                if (state == State.TRANSIENT) {
                    orient.update(targetHeading, currentHeading);
                    longitudinal.update(targetDistance, currentDistance);
                    drivebase.setPowers(longitudinal.correction - orient.correction, longitudinal.correction + orient.correction);                    drivebase.update();
                } else if (state == State.CONVERGED) {
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
        state = State.TRANSIENT;
    }

    public double getDistToTarget(){
        currentDistance = (drivebase.getLeftEncoder() + drivebase.getRightEncoder())/2;
        currentHeading = odometer.heading;
        return distance(targetHeading, targetDistance, currentHeading, currentDistance);
    }

    public void followPath(ArrayList<PointEx> path, double radius){

        PointEx lastPoint = path.get(path.size()-1); //Last point in the ArrayList

        PointEx targetPoint = PurePursuit.getTargetPoint(odometer.x, odometer.y, radius, path);
        // PathingAgent uses line-circle intersect to get 0,1, or 2 points, then picks whichever point is closest to the current "goal" RobotPoint

        if(targetPoint.speed == 404){ // If there are no intersections
            // Trigger fail-safe to regain the path
            targetPoint = PurePursuit.getFailsafePoint(odometer.x, odometer.y, path);
        }

        // Checking if the robot is within a certain distance of the "last" point
        double distanceX = lastPoint.x - odometer.x;
        double distanceY = lastPoint.y - odometer.y;
        double totalDistance = Math.hypot(distanceX, distanceY);

        if(totalDistance < 10){ // End loop if you are within 5 cm of the last point
            break;
        }

        //Now that the robot knows where to go (targetPoint, returned by PathingAgent), the following code handles motor powers.
        double xDist, yDist, distance, heading;
        double targSpeed, scale, targVX, targVY;

        xDist = targetPoint.x - odometer.x;
        yDist = targetPoint.y - odometer.y;
        distance = Math.hypot(xDist, yDist);
        heading = odometer.heading;

        targSpeed = Math.abs(targetPoint.speed);
        scale = targSpeed / distance;

        targVX = xDist * scale;
        targVY = yDist * scale;

        orient.update(targetPoint.heading, heading); //targetPoint.heading is goal heading

        setGlobalVelocity(targVX, targVY, orient.correction);

    }

    public double getLongitudinalCorrect(){
        return longitudinal.correction;
    }
    public double getHeadingCorrect(){
        return orient.correction;
    }
    public double getHeading() {return odometer.heading;}

}
*/