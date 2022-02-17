package org.firstinspires.ftc.teamcode.Controllers;

import org.firstinspires.ftc.teamcode.MathFunctions.MyMath;

public class TurretController extends Controller {

    private PIDF backBone;
    private double rotInertia;
    private final double maxRotInertia;

    private final double minP = 0.003; // Proportional gain when fully in
    private final double maxP = 0.0075; // Proportional gain when fully out

    private final double minI = 0.0003;
    private final double maxI = 0.0007;

    private final double minD = 0.0045;
    private final double maxD = 0.0075;

    private final double minILimit = 0.024;
    private final double maxILimit = 0.027;

    public TurretController(double slideLimit) {
        backBone = new PIDF(0,0,0,0, 0,0.4, 0);
        maxRotInertia = Math.pow(slideLimit, 2);
    }

    public void update(double target, double current, double slideLength) {
        rotInertia = Math.pow(slideLength, 2)/maxRotInertia;
        double kP = MyMath.lerp(rotInertia, minP, maxP);
        double kI = MyMath.lerp(rotInertia, minI, maxI);
        double kD = MyMath.lerp(rotInertia, minD, maxD);
        double I = MyMath.lerp(rotInertia, minILimit, maxILimit);
        backBone.updateGains(kP, kI, kD, 0.001, I);
        backBone.update(target, current);
        correction = backBone.correction;
        if((target - current) > 2000) {
            correction = 0.4;
        }else if((target - current) < -2000) {
            correction = -0.4;
        }
    }
}
