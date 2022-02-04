package org.firstinspires.ftc.teamcode.MathFunctions;

import java.util.ArrayList;
import static java.lang.Math.*;

public class MyMath {

    public static ArrayList<PointEx> lineCircleIntersection(double circleX, double circleY, double r,
                                                               double lineX1, double lineY1,
                                                               double lineX2, double lineY2){
        // Make sure the points don't exactly line up so the slopes work
        if(Math.abs(lineY1- lineY2) < 0.003){

            lineY1 = lineY2 + 0.003;

        }
        if(Math.abs(lineX1- lineX2) < 0.003){

            lineX1 = lineX2 + 0.003;

        }

        // Calculate the slope of the line
        double m1 = (lineY2 - lineY1)/(lineX2-lineX1);

        // The first coefficient in the quadratic
        double quadraticA = 1.0 + pow(m1,2);

        // Shift one of the line's points so it is relative to the circle
        double x1 = lineX1-circleX;
        double y1 = lineY1-circleY;


        // The second coefficient in the quadratic
        double quadraticB = (2.0 * m1 * y1) - (2.0 * pow(m1,2) * x1);

        // The third coefficient in the quadratic
        double quadraticC = ((pow(m1,2)*pow(x1,2)) - (2.0*y1*m1*x1) + pow(y1,2)-pow(r,2));


        ArrayList<PointEx> allPoints = new ArrayList<>();

        // This may give an error so we use a try catch
        try{
            // Now solve the quadratic equation given the coefficients
            double discriminant = sqrt(pow(quadraticB,2) - (4.0 * quadraticA * quadraticC));
            double xRoot1 = (-quadraticB + discriminant)/(2.0*quadraticA);

            // We know the line equation so plug into that to get root
            double yRoot1 = m1 * (xRoot1 - x1) + y1;

            // Add back in translations
            xRoot1 += circleX;
            yRoot1 += circleY;

            // Make sure it was within range of the segment
            double minX = min(lineX1, lineX2);
            double maxX = max(lineX1, lineX2);

            if(xRoot1 > minX && xRoot1 < maxX){

                allPoints.add(new PointEx(xRoot1, yRoot1, 0));

            }

            // Do the same for the other root
            double xRoot2 = (-quadraticB - discriminant)/(2.0*quadraticA);

            double yRoot2 = m1 * (xRoot2 - x1) + y1;
            // Now we can add back in translations
            xRoot2 += circleX;
            yRoot2 += circleY;

            // Make sure it was within range of the segment
            if(xRoot2 > minX && xRoot2 < maxX){

                allPoints.add(new PointEx(xRoot2, yRoot2, 0));

            }

        }catch(Exception e){
            // There are no real roots, and therefore no intersections
        }
        return allPoints;
    }

    public static PointEx lineLineIntersection(PointEx line1P1, PointEx line1P2, PointEx line2P1, PointEx line2P2) {
        double slope1 = slope(line1P1, line1P2);
        double slope2 = slope(line2P1, line2P2);
        if(slope1 == slope2) {
            return null;
        }else {
            double x = (line1P1.x * slope1 - line1P1.y - line2P1.x * slope2 + line2P1.y) / (slope1 - slope2);
            double y = slope1 * (x - line1P2.x) + line1P2.y;
            return new PointEx(x, y, 0);
        }
    }

    public static PointEx perpendicularBisector(PointEx P1, PointEx P2) {
        if(slope(P1, P2) != 0) {
            double perpSlope = -1/slope(P1, P2);
            PointEx midPt = midPoint(P1, P2);
            midPt.heading = perpSlope;
            return midPt;
        }else {
            return null;
        }
    }

    public static double slope(PointEx P1, PointEx P2) {
        return (P2.y - P1.y)/(P2.x - P1.x);
    }

    public static ArrayList<PointEx> circleCircleIntersection(double x1, double y1, double r1, double x2, double y2, double r2) {
        double xCoeff = -2*(x1-x2);
        double yCoeff = -2*(y1-y2);
        double constant = (Math.pow(r1, 2) - Math.pow(r2, 2)) - (Math.pow(x1, 2) - Math.pow(x2, 2)) - (Math.pow(y1, 2) - Math.pow(y2, 2));

        double lineX1 = constant/xCoeff;
        double lineY2 = constant/yCoeff;

        return lineCircleIntersection(x1, y1, r1, lineX1, 0, 0, lineY2);
    }

    public static double[] cubicRoots(double a, double b, double c, double d) {

        double A=b/a;
        double B=c/a;
        double C=d/a;

        double Q, R, D, S, T, Im;

        Q = (3*B - Math.pow(A, 2))/9;
        R = (9*A*B - 27*C - 2*Math.pow(A, 3))/54;
        D = Math.pow(Q, 3) + Math.pow(R, 2);    // polynomial discriminant

        double[] t = new double[3];

        if (D >= 0) { // complex or duplicate roots
            S = Math.signum((float)(R + Math.sqrt(D))*Math.pow(Math.abs(R + Math.sqrt(D)),(1.0/3)));
            T = Math.signum((float)(R - Math.sqrt(D))*Math.pow(Math.abs(R - Math.sqrt(D)),(1.0/3)));

            t[0] = -A/3 + (S + T);                    // real root
            t[1] = -A/3 - (S + T)/2;                  // real part of complex root
            t[2] = -A/3 - (S + T)/2;                  // real part of complex root
            Im = Math.abs(Math.sqrt(3)*(S - T)/2);    // complex part of root pair

            // discard complex roots
            if (Im!=0) {
                t[1]=-1;
                t[2]=-1;
            }
        }else { // distinct real roots
            double th = Math.acos(R/Math.sqrt(-Math.pow(Q, 3)));

            t[0] = 2*Math.sqrt(-Q)*Math.cos(th/3) - A/3;
            t[1] = 2*Math.sqrt(-Q)*Math.cos((th + 2*Math.PI)/3) - A/3;
            t[2] = 2*Math.sqrt(-Q)*Math.cos((th + 4*Math.PI)/3) - A/3;
            Im = 0.0;
        }

        // discard out of spec roots
        for (int i=0;i<3;i++) {
            if (t[i]<0 || t[i]>1.0) t[i]=-1;
        }
        return t;
    }

    public static PointEx computeIntersections(BezierCurve bezier, PointEx lineP1, PointEx lineP2) {
        double[] X = new double[2];

        double[] px = {bezier.point0start.x, bezier.point1corner.x, bezier.point2corner.x, bezier.point3end.x};
        double[] py = {bezier.point0start.y, bezier.point1corner.y, bezier.point2corner.y, bezier.point3end.y};

        double[] lx = {lineP1.x, lineP2.x};
        double[] ly = {lineP1.y, lineP2.y};

        double A=ly[1]-ly[0];	    //A=y2-y1
        double B=lx[0]-lx[1];	    //B=x1-x2
        double C=lx[0]*(ly[0]-ly[1]) + ly[0]*(lx[1]-lx[0]);	//C=x1*(y1-y2)+y1*(x2-x1)

        double[] bx = bezierCoeffs(px[0],px[1],px[2],px[3]);
        double[] by = bezierCoeffs(py[0],py[1],py[2],py[3]);

        double[] P = new double[4];
        P[0] = A*bx[0]+B*by[0];		//t^3
        P[1] = A*bx[1]+B*by[1];		//t^2
        P[2] = A*bx[2]+B*by[2];		//t
        P[3] = A*bx[3]+B*by[3] + C;	//1

        double[] r = cubicRoots(P[0], P[1], P[2], P[3]);

        // verify the roots are in bounds of the linear segment
        for (int i=0;i<3;i++) {
            double t = r[i];

            X[0]=bx[0]*t*t*t+bx[1]*t*t+bx[2]*t+bx[3];
            X[1]=by[0]*t*t*t+by[1]*t*t+by[2]*t+by[3];

        // above is intersection point assuming infinitely long line segment, make sure we are also in bounds of the line
            double s;
            if ((lx[1]-lx[0])!=0)           // if not vertical line
                s=(X[0]-lx[0])/(lx[1]-lx[0]);
            else
                s=(X[1]-ly[0])/(ly[1]-ly[0]);

            // in bounds?
            if (t<0 || t>1.0 || s<0 || s>1.0)
            {
                X[0]=-100;  // move off screen
                X[1]=-100;
            }

            // move intersection point
            //I[i].setAttributeNS(null,"cx",X[0]);
            //I[i].setAttributeNS(null,"cy",X[1]);
        }

        return new PointEx(0,0,0);
    }

    private static double[] bezierCoeffs(double P0, double P1, double P2, double P3) {
        double[] Z = new double[4];
        Z[0] = -P0 + 3*P1 + -3*P2 + P3;
        Z[1] = 3*P0 - 6*P1 + 3*P2;
        Z[2] = -3*P0 + 3*P1;
        Z[3] = P0;
        return Z;
    }

    public static double sine(double theta){
        return sin(toRadians(theta));
    }

    public static double cosine(double theta){
        return cos(toRadians(theta));
    }

    public static double distance(double x1, double y1, double x2, double y2) {
        return Math.hypot((x1-x2), (y1-y2));
    }

    public static double distance(PointEx p1, PointEx p2) {
        return Math.hypot((p1.x-p2.x), (p1.y-p2.y));
    }

    public static double clip(double value, double min, double max) {
        if(value < min) {
            value = min;
        }else if(value > max) {
            value = max;
        }
        return value;
    }

    public static PointEx midPoint(PointEx p1, PointEx p2) {
        return new PointEx((p1.x + p2.x)/2,(p1.y + p2.y)/2,(p1.heading + p2.heading)/2);
    }

    public static double[] midPoint(double x1, double y1, double x2, double y2) {
        double[] midP = {(x1+x2)/2, (y1+y2)/2};
        return midP.clone();
    }

    public static double bound(double value, double upper, double lower) {
        if(upper < lower) {
            double tmp = lower;
            lower = upper;
            upper = tmp;
        }
        if(value >= upper) value = upper;
        if(value <= lower) value = lower;
        return value;
    }

}
