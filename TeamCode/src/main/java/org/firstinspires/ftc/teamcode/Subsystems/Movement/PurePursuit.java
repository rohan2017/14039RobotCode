package org.firstinspires.ftc.teamcode.Subsystems.Movement;

import org.firstinspires.ftc.teamcode.MathFunctions.*;
import java.util.ArrayList;

public class PurePursuit {

    public static PointEx getTargetPoint(double robotX, double robotY, double radius, ArrayList<PointEx> path){

        // Initialized like this to create a condition where there is no intersection
        PointEx targetPoint = new PointEx(0,0,0); // Default case
        targetPoint.speed = 40404;

        for(int i=0; i<path.size()-1; i++) {

            PointEx pathPoint1 = path.get(i);
            PointEx pathPoint2 = path.get(i+1);

            ArrayList<PointEx> candidates = MyMath.lineCircleIntersection(robotX, robotY,
                    radius, pathPoint1.x, pathPoint1.y, pathPoint2.x, pathPoint2.y);

            if(candidates.size() == 1){

                // If the robot is on the last segment and only gets 1 line/circle intersection, it
                // also checks the final point as a candidate for targetPoint. If final point is
                // closer to the robot than other candidate, it chooses that as target
                targetPoint = candidates.get(0);

                if (i == path.size()-2){ //if you are working with the last segment

                    double endPointDistance = Math.hypot((robotX - pathPoint2.x), (robotY - pathPoint2.y)); //distance from end of path to robot
                    if (endPointDistance < radius){
                        targetPoint = pathPoint2; //if the end point is closer than the circle intersect, then choose it instead
                    }
                }

            }else if(candidates.size() == 2){
                PointEx point1 = candidates.get(0);
                PointEx point2 = candidates.get(1);

                // Figure out which point is closer to the next point (further along the path)
                double distance1 = Math.hypot((pathPoint2.x-point1.x), (pathPoint2.y-point1.y));
                double distance2 = Math.hypot((pathPoint2.x-point2.x), (pathPoint2.y-point2.y));

                if(distance1 > distance2){ //Choose point 2
                    targetPoint = point2;
                }else{ //Choose point 1
                    targetPoint = point1;
                }
            }

            // Calculating the desired speed and heading of the robot
            double segmentDistance = Math.hypot((pathPoint1.x - pathPoint2.x), (pathPoint1.y - pathPoint2.y));
            double distanceToOne = Math.hypot((pathPoint1.x - targetPoint.x), (pathPoint1.y - targetPoint.y));
            double distanceToTwo = segmentDistance - distanceToOne;

            targetPoint.heading = (pathPoint1.heading * distanceToOne + pathPoint2.heading * distanceToTwo) / (segmentDistance); //Lerp of point 1 and point 2 heading
            targetPoint.speed = (pathPoint1.speed * distanceToOne + pathPoint2.speed * distanceToTwo) / (segmentDistance); //Lerp of point 1 and point 2 speeds

        }

        return targetPoint;
    }

    public static PointEx getFailsafePoint(double robotX, double robotY, ArrayList<PointEx> path){
        double leastDistance = 500;
        for(int i=0; i<path.size(); i++){
            double distance = Math.hypot((robotX - path.get(i).x), (robotY - path.get(i).y));
            if(distance < leastDistance){
                leastDistance = distance;
            }
        }

        return getTargetPoint(robotX, robotY, leastDistance+10, path);
    }

    public static void generateTrajectoryH(ArrayList<PointEx> path, VelocityCurve veloCurve) {
        double totalDistance = 0;
        for(int i=0; i<path.size()-1; i++) {
            PointEx pathPoint1 = path.get(i);
            PointEx pathPoint2 = path.get(i+1);
            totalDistance += MyMath.distance(pathPoint1, pathPoint2);
        }
        veloCurve.setDistance(totalDistance);
        double distance = 0;
        path.get(0).speed = veloCurve.getTargetVelocity(0);
        for(int i=0; i<path.size()-1; i++) {
            PointEx pathPoint1 = path.get(i);
            PointEx pathPoint2 = path.get(i+1);
            distance += MyMath.distance(pathPoint1, pathPoint2);
            pathPoint1.heading = Math.toDegrees(Math.atan2(pathPoint2.y-pathPoint1.y, pathPoint2.x-pathPoint1.x))-90;
            pathPoint2.speed = veloCurve.getTargetVelocity(distance);
        }
        path.get(path.size()-1).heading = path.get(path.size()-2).heading;
    }

    public static void generateTrajectory(ArrayList<PointEx> path, VelocityCurve veloCurve) {
        double totalDistance = 0;
        for(int i=0; i<path.size()-1; i++) {
            PointEx pathPoint1 = path.get(i);
            PointEx pathPoint2 = path.get(i+1);
            totalDistance += MyMath.distance(pathPoint1, pathPoint2);
        }
        veloCurve.setDistance(totalDistance);
        double distance = 0;
        path.get(0).speed = veloCurve.getTargetVelocity(0);
        for(int i=0; i<path.size()-1; i++) {
            PointEx pathPoint1 = path.get(i);
            PointEx pathPoint2 = path.get(i+1);
            distance += MyMath.distance(pathPoint1, pathPoint2);
            pathPoint2.speed = veloCurve.getTargetVelocity(distance);
        }

    }

}
