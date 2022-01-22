package org.firstinspires.ftc.teamcode.Subsystems.Movement;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.MathFunctions.MyMath;
import org.firstinspires.ftc.teamcode.MathFunctions.PointEx;
import org.firstinspires.ftc.teamcode.Subsystems.Localization.Odometer;
import org.firstinspires.ftc.teamcode.Subsystems.Movement.Drivebases.DummyTankDrive;
import org.firstinspires.ftc.teamcode.Subsystems.State;

import java.util.ArrayList;

public class MovementTankDrive extends Movement {

    private DummyTankDrive drivebase;

    private double leftSpeed, rightSpeed;
    private PointEx targetPoint;
    private ArrayList<PointEx> targetPath;
    private double pathRadius;

    public enum DriveMode {
        GoToPoint,
        FollowPath,
        Turn,
        Stopped
    }

    DriveMode mode;

    public MovementTankDrive(LinearOpMode opMode, DummyTankDrive drivetrain, Odometer odometer) {
        super(opMode, odometer);
        this.drivebase = drivetrain;
    }

    public void initialize() {
        state = State.IDLE;
        leftSpeed = 0;
        rightSpeed = 0;
        mode = DriveMode.GoToPoint;
        PointEx targetPoint = new PointEx(0,0,0);
    }

    public void update() {
        if(opMode.opModeIsActive()) {
            if(state != State.IDLE) {
                // State determination
                if(MyMath.distance(targetPoint, new PointEx(odometer.x, odometer.y, 0)) < 5 && Math.abs(odometer.heading-targetPoint.heading) < 5) {
                    state = State.CONVERGED;
                }else {
                    state = State.TRANSIENT;
                }

                // Actions
                if (state == State.TRANSIENT) {
                    switch (mode) {
                        case FollowPath:
                            if(followTrajectory()) {
                                drivebase.setPowers(leftSpeed*targetPoint.speed, rightSpeed*targetPoint.speed);
                            }else {
                                mode = DriveMode.Stopped;
                            }
                            break;
                        case GoToPoint:
                            if(updateTargetPointArc()) {
                                drivebase.setPowers(leftSpeed*targetPoint.speed, rightSpeed*targetPoint.speed);
                            }else {
                                mode = DriveMode.Stopped;
                            }
                            break;
                        case Turn:
                            drivebase.setPowers(leftSpeed, rightSpeed);
                            break;
                        case Stopped:
                            drivebase.freeze();
                            break;
                    }
                } else if (state == State.CONVERGED) {
                    drivebase.freeze();
                }
            }else {
                drivebase.freeze();
            }
            odometer.update();
        }
    }

    public void setTarget(ArrayList<PointEx> path, double radius) {
        mode = DriveMode.FollowPath;
        pathRadius = radius;
        targetPath = path;
    }

    public void setTarget(PointEx point) {
        targetPoint = point;
    }

    private boolean followTrajectory() {

        PointEx lastPoint = targetPath.get(targetPath.size()-1); //Last point in the ArrayList

        targetPoint = PurePursuit.getTargetPoint(odometer.x, odometer.y, pathRadius, targetPath);
        // Uses line-circle intersect to get 0,1, or 2 points, then picks whichever point is closest to the current "goal" point

        if(targetPoint.speed == 40404){ // If there are no intersections
            // Trigger fail-safe to regain the path
            targetPoint = PurePursuit.getFailsafePoint(odometer.x, odometer.y, targetPath);
        }

        // Checking if the robot is within a certain distance of the "last" point
        double totalDistance = MyMath.distance(lastPoint, new PointEx(odometer.x, odometer.y, 0));
        return updateTargetPointArc() && (totalDistance > pathRadius);

    }

    private boolean updateTargetPointArc() { // moves in a circular arc to end up at a target point
        PointEx currentPosition = new PointEx(odometer.x, odometer.y, odometer.heading+Math.PI/2);
        double relHeading = currentPosition.heading - Math.atan2((targetPoint.y - odometer.y), (targetPoint.x - odometer.x));
        if(Math.abs(relHeading) < 0.01) { // If point is straight ahead
            leftSpeed = MyMath.distance(currentPosition, targetPoint);
            rightSpeed = leftSpeed;
            return true;
        }else if(Math.abs(relHeading)%Math.PI < 0.01) { // or behind
            leftSpeed = -MyMath.distance(currentPosition, targetPoint);
            rightSpeed = -leftSpeed;
            return true;
        }else { // If not, then arc to it
            PointEx perp1 = MyMath.perpendicularBisector(currentPosition, targetPoint);
            if(perp1 != null) {
                PointEx perp2 = new PointEx(perp1.x+1, perp1.y+perp1.heading, 0);
                double perpHeadingSlope = -1/Math.tan(currentPosition.heading);
                PointEx positionPerp = new PointEx(currentPosition.x+1, currentPosition.y+perpHeadingSlope, 0);
                PointEx turnCenter = MyMath.lineLineIntersection(currentPosition, positionPerp, perp1, perp2);
                if(turnCenter != null) {
                    double radius = MyMath.distance(currentPosition, turnCenter);
                    double theta = Math.acos(1 - Math.pow(MyMath.distance(currentPosition, targetPoint),2)/(2*radius*radius));
                    double innerTrack = (radius-(drivebase.wheelBase/2))*theta;
                    double midTrack = radius*theta;
                    double outerTrack = (radius+(drivebase.wheelBase/2))*theta;
                    double multiplier = Math.abs(relHeading) > Math.PI/2 ? -1 : 1;
                    if(Math.signum(turnCenter.y-currentPosition.y)/(turnCenter.x-currentPosition.x) == Math.signum(perpHeadingSlope)) {
                        leftSpeed = outerTrack*multiplier/midTrack;
                        rightSpeed = innerTrack*multiplier/midTrack;
                    }else {
                        leftSpeed = innerTrack*multiplier/midTrack;
                        rightSpeed = outerTrack*multiplier/midTrack;
                    }
                    return true;
                }
            }
        }
        return false;
    }

    private void updateTargetPointSimple(PointEx point) {
        PointEx currentPosition = new PointEx(odometer.x, odometer.y, odometer.heading+Math.PI/2);
        double relHeading = currentPosition.heading - Math.atan2((point.y - odometer.y), (point.x - odometer.x));
        if(relHeading < 0) {
            leftSpeed = (1 + relHeading*drivebase.wheelBase/2)/(1 - relHeading*drivebase.wheelBase/2);
            rightSpeed = 1;
        }else {
            rightSpeed = (1 - relHeading*drivebase.wheelBase/2)/(1 + relHeading*drivebase.wheelBase/2);
            leftSpeed = 1;
        }
    }

}