package org.firstinspires.ftc.teamcode.Controllers;

import org.firstinspires.ftc.teamcode.MathFunctions.MyMath;

public class TurretController extends Controller {

    private PIDF backBone;
    private double rotInertia;
    private final double maxRotInertia;

    private final double minP = 0.004;
    private final double maxP = 0.007;

    private final double minI = 0.0001;
    private final double maxI = 0.0003;
    
    private final double minD = 0.0008;
    private final double maxD = 0.003;

    public TurretController(double slideLimit) {
        backBone = new PIDF(0.004,0,0.0005,0.01, 0,0.4, 0);
        maxRotInertia = Math.pow(slideLimit, 2);
    }

    public void update(double slideLength) {
        rotInertia = Math.pow(slideLength, 2)/maxRotInertia;
        double kP = MyMath.lerp(rotInertia, minP, maxP);

    }
}
