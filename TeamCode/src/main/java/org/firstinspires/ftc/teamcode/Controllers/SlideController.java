package org.firstinspires.ftc.teamcode.Controllers;

public class SlideController extends Controller {

    private PIDF backbone;
    private final double maxSlidePwr = 0.8;

    public SlideController() {
        backbone = new PIDF(0.005, 0.00001, 0.001, 0.002, 0.08, maxSlidePwr, 0);
    }

    public void update(double target, double current, boolean limitSwitch) {
        backbone.update(target, current);
        if(target < 1 && !limitSwitch) {
            if(current > 200) {
                correction = -maxSlidePwr;
            }else if(current < 3) {
                correction = 0;
            }else {
                correction = -0.4;
            }
        }else if(target - current > 150) {
            correction = maxSlidePwr;
        }else if(target - current < -150) {
            correction = -maxSlidePwr;
        }else {
            correction = backbone.correction;
        }
    }

}
