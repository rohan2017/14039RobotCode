package org.firstinspires.ftc.teamcode.Controllers;

public class SlideController extends Controller {

    private PIDF backbone;
    private final double maxSlidePwr = 0.8;

    public SlideController() {
        backbone = new PIDF(0.005, 0.00001, 0.002, 0.002, 0.06, maxSlidePwr, 0);
    }

    public void update(double target, double current, boolean limitSwitch, double tiltAngle) {
        if(tiltAngle < 10 && target > 100) {
            backbone.updateGains(0.004, 0.00001, 0.005, 0.002, 0.03);
        }else {
            backbone.updateGains(0.005, 0.00001, 0.002, 0.002, 0.06);
        }
        backbone.update(target, current);
        if(target < 1 && !limitSwitch) {
            if(current > 200) {
                correction = -maxSlidePwr;
            } else {
                correction = -0.4;
            }
        }else if(target < 1 && limitSwitch) {
            correction = 0;
        }else if(target - current > 150) {
            correction = maxSlidePwr;
        }else if(target - current < -150) {
            correction = -maxSlidePwr;
        }else {
            correction = backbone.correction;
            if(Math.abs(target - current) < 30) {
                correction *= 0.2;
            }
        }
    }

}
