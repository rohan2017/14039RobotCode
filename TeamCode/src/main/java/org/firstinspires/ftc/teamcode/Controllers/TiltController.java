package org.firstinspires.ftc.teamcode.Controllers;

import org.firstinspires.ftc.teamcode.MathFunctions.MyMath;

public class TiltController extends Controller {

    private PIDF backBone;
    private double rotInertia;
    private final double maxRotInertia;

    private final double minP = 0.035; // Proportional gain when fully in
    private final double maxP = 0.11; // Proportional gain when fully out

    private final double minI = 0.0001;
    private final double maxI = 0;

    private final double minD = 0.0008;
    private final double maxD = 0.003;

    private final double minILimit = 0.05;
    private final double maxILimit = 0.03;

    private boolean flag;
    private double lastSlides;

    public TiltController(double slideLimit) {
        backBone = new PIDF(0,0,0,0, 0,0.4,0);
        maxRotInertia = Math.pow(slideLimit, 2);
        flag = true;
    }

    public void update(double target, double current, double slideLength) {
        if(flag) {
            lastSlides = slideLength;
            flag = false;
        }
        rotInertia = Math.pow(slideLength, 2)/maxRotInertia;
        double kP = MyMath.lerp(rotInertia, minP, maxP);
        double kI = MyMath.lerp(rotInertia, minI, maxI);
        double kD = MyMath.lerp(rotInertia, minD, maxD);
        double I = MyMath.lerp(rotInertia, minILimit, maxILimit);
        backBone.updateGains(kP, kI, kD, 0.01, I);
        backBone.update(target, current);
        correction = backBone.correction;
        double slideFF = rotInertia * (slideLength-lastSlides) * 0.0003;
        correction += slideFF;
        if((target - current) > 2000) {
            correction = 0.4;
        }else if((target - current) < -2000) {
            correction = -0.4;
        }
        lastSlides = slideLength;
    }
}
