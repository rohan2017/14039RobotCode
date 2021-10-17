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

    function cubicRoots(P)
    {
        var a=P[0];
        var b=P[1];
        var c=P[2];
        var d=P[3];

        var A=b/a;
        var B=c/a;
        var C=d/a;

        var Q, R, D, S, T, Im;

        var Q = (3*B - Math.pow(A, 2))/9;
        var R = (9*A*B - 27*C - 2*Math.pow(A, 3))/54;
        var D = Math.pow(Q, 3) + Math.pow(R, 2);    // polynomial discriminant

        var t=Array();

        if (D >= 0)                                 // complex or duplicate roots
        {
            var S = sgn(R + Math.sqrt(D))*Math.pow(Math.abs(R + Math.sqrt(D)),(1/3));
            var T = sgn(R - Math.sqrt(D))*Math.pow(Math.abs(R - Math.sqrt(D)),(1/3));

            t[0] = -A/3 + (S + T);                    // real root
            t[1] = -A/3 - (S + T)/2;                  // real part of complex root
            t[2] = -A/3 - (S + T)/2;                  // real part of complex root
            Im = Math.abs(Math.sqrt(3)*(S - T)/2);    // complex part of root pair

            /*discard complex roots*/
            if (Im!=0)
            {
                t[1]=-1;
                t[2]=-1;
            }

        }
        else                                          // distinct real roots
        {
            var th = Math.acos(R/Math.sqrt(-Math.pow(Q, 3)));

            t[0] = 2*Math.sqrt(-Q)*Math.cos(th/3) - A/3;
            t[1] = 2*Math.sqrt(-Q)*Math.cos((th + 2*Math.PI)/3) - A/3;
            t[2] = 2*Math.sqrt(-Q)*Math.cos((th + 4*Math.PI)/3) - A/3;
            Im = 0.0;
        }

        /*discard out of spec roots*/
        for (var i=0;i<3;i++)
            if (t[i]<0 || t[i]>1.0) t[i]=-1;

        /*sort but place -1 at the end*/
        t=sortSpecial(t);

        console.log(t[0]+" "+t[1]+" "+t[2]);
        return t;
    }

    function computeIntersections(px,py,lx,ly)
    {
        var X=Array();

        var A=ly[1]-ly[0];	    //A=y2-y1
        var B=lx[0]-lx[1];	    //B=x1-x2
        var C=lx[0]*(ly[0]-ly[1]) +
                ly[0]*(lx[1]-lx[0]);	//C=x1*(y1-y2)+y1*(x2-x1)

        var bx = bezierCoeffs(px[0],px[1],px[2],px[3]);
        var by = bezierCoeffs(py[0],py[1],py[2],py[3]);

        var P = Array();
        P[0] = A*bx[0]+B*by[0];		/*t^3*/
        P[1] = A*bx[1]+B*by[1];		/*t^2*/
        P[2] = A*bx[2]+B*by[2];		/*t*/
        P[3] = A*bx[3]+B*by[3] + C;	/*1*/

        var r=cubicRoots(P);

        /*verify the roots are in bounds of the linear segment*/
        for (var i=0;i<3;i++)
        {
            t=r[i];

            X[0]=bx[0]*t*t*t+bx[1]*t*t+bx[2]*t+bx[3];
            X[1]=by[0]*t*t*t+by[1]*t*t+by[2]*t+by[3];

        /*above is intersection point assuming infinitely long line segment,
          make sure we are also in bounds of the line*/
            var s;
            if ((lx[1]-lx[0])!=0)           /*if not vertical line*/
                s=(X[0]-lx[0])/(lx[1]-lx[0]);
            else
                s=(X[1]-ly[0])/(ly[1]-ly[0]);

            /*in bounds?*/
            if (t<0 || t>1.0 || s<0 || s>1.0)
            {
                X[0]=-100;  /*move off screen*/
                X[1]=-100;
            }

            /*move intersection point*/
            I[i].setAttributeNS(null,"cx",X[0]);
            I[i].setAttributeNS(null,"cy",X[1]);
        }
    }
}
