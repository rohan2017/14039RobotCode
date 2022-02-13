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
    public PointEx targetPoint, currentPosition;
    private ArrayList<PointEx> targetPath;
    private double pathRadius;

    private double distanceThreshold = 4;
    private double headingThreshold = 6;

    private PID orient;
    private PID longitudinal;

    public enum DriveMode {
        GoToPoint,
        FollowPath,
        GoToPointSimple,
        Stopped
    }

    DriveMode mode;

    public MovementTankDrive(LinearOpMode opMode, DummyTankDrive drivetrain, Odometer odometer) {
        super(opMode, odometer);
        this.drivebase = drivetrain;
        orient = new PID(0.005,0,0.001,0,0.2,0);
        longitudinal = new PID(0.1,0,0.08,0,0.5,0);
    }

    public void initialize() {
        leftSpeed = 0;
        rightSpeed = 0;
        mode = DriveMode.GoToPoint;
        targetPoint = new PointEx(0,0,0);
        currentPosition = new PointEx(odometer.x, odometer.y, odometer.heading);
        state = State.TRANSIENT;
    }

    public void update() {
        if(opMode.opModeIsActive()) {
            currentPosition.x = odometer.x;
            currentPosition.y = odometer.y;
            currentPosition.heading = odometer.heading;

            if(state != State.IDLE) {
                // State determination
                if(MyMath.distance(targetPoint, new PointEx(odometer.x, odometer.y, 0)) < distanceThreshold && Math.abs(odometer.heading-targetPoint.heading) < headingThreshold) {
                    state = State.CONVERGED;
                }else {
                    state = State.TRANSIENT;
                }

                // Actions
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
                    case GoToPointSimple:
                        updateTargetPointSimple();
                        drivebase.setPowers(leftSpeed, rightSpeed);
                        break;
                    case Stopped:
                        drivebase.freeze();
                        break;
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
        state = State.TRANSIENT;
    }

    public void setTarget(PointEx point) {
        targetPoint = point;
        targetPoint.heading = point.heading;
        mode = DriveMode.GoToPointSimple;
        state = State.TRANSIENT;
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
        updateTargetPointSimple();
        return (totalDistance > pathRadius);

    }

    private boolean updateTargetPointArc() { // moves in a circular arc to end up at a target point
        double relHeading = currentPosition.heading+90 - Math.toDegrees(Math.atan2((targetPoint.y - odometer.y), (targetPoint.x - odometer.x)));
        if(Math.abs(relHeading) < 0.01) { // If point is straight ahead
            leftSpeed = MyMath.distance(currentPosition, targetPoint);
            rightSpeed = leftSpeed;
            return true;
        }else if(Math.abs(relHeading)%180 < 0.01) { // or behind
            leftSpeed = -MyMath.distance(currentPosition, targetPoint);
            rightSpeed = -leftSpeed;
            return true;
        }else { // If not, then arc to it
            PointEx perp1 = MyMath.perpendicularBisector(currentPosition, targetPoint);
            if(perp1 != null) {
                PointEx perp2 = new PointEx(perp1.x+1, perp1.y+perp1.heading, 0);
                double perpHeadingSlope = -1/Math.tan(Math.toRadians(currentPosition.heading)+Math.PI/2);
                PointEx positionPerp = new PointEx(currentPosition.x+1, currentPosition.y+perpHeadingSlope, 0);
                PointEx turnCenter = MyMath.lineLineIntersection(currentPosition, positionPerp, perp1, perp2);
                if(turnCenter != null) {
                    double radius = MyMath.distance(currentPosition, turnCenter);
                    double theta = Math.acos(1 - Math.pow(MyMath.distance(currentPosition, targetPoint),2)/(2*radius*radius));
                    double innerTrack = (radius-(drivebase.wheelBase/2))*theta;
                    double midTrack = radius*theta;
                    double outerTrack = (radius+(drivebase.wheelBase/2))*theta;
                    double multiplier = Math.abs(relHeading) > 90 ? -1 : 1;
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

    private void updateTargetPointSimple() {
        double dir;
        double pointHeading = (Math.toDegrees(Math.atan2((targetPoint.y - odometer.y), (targetPoint.x - odometer.x))) - 90)%360;
        if((pointHeading - currentPosition.heading%360) < 90 && (pointHeading - currentPosition.heading%360) > -90) {
            dir = 1;
        }else {
            pointHeading = (pointHeading + 180)%360;
            dir = -1;
        }
        double distance = MyMath.distance(targetPoint, currentPosition);
        if(distance > distanceThreshold) {
            orient.update(pointHeading, currentPosition.heading%360);
            longitudinal.update(0, distance);
            double longCorrect = dir*longitudinal.correction;
            if(Math.abs(orient.error) > 15) {
                longCorrect *= 4*Math.pow(Math.abs(1/orient.error), 0.5);
            }
            double slow = 1;
            if(distance < 20 && Math.abs(orient.error) < 15) {
                slow = 0.5;
            }
            rightSpeed = (-longCorrect + orient.correction)*slow;
            leftSpeed = (-longCorrect - orient.correction)*slow;
        }else {
            orient.update(targetPoint.heading, currentPosition.heading);
            rightSpeed = orient.correction;
            leftSpeed = -orient.correction;
        }

    }

}