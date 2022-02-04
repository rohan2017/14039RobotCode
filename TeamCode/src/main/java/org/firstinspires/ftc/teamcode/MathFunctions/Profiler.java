package org.firstinspires.ftc.teamcode.MathFunctions;

import org.firstinspires.ftc.teamcode.Subsystems.Timer;

public class Profiler {

    public double output;
    public double target;
    private double startTime;
    private double startReading;

    private double speed;
    private double timeToTarget;
    private final int mode;

    private Interpolation type;
    private Timer timer;

    public Profiler(Timer timer, double speed, Interpolation type) {
        this.timer = timer;
        this.type = type;
        this.speed = speed; // units per millisecond
        mode = 0;
    }

    public Profiler(Timer timer, Interpolation type, double timeToTarget) {
        this.timer = timer;
        this.type = type;
        this.timeToTarget = timeToTarget;
        mode = 1;
    }

    public void setValue(double value) {
        output = value;
    }

    public void setTarget(double target) {
        this.target = target;
        startTime = timer.getTime();
        startReading = output;
    }

    public void update() {
        switch(type) {
            case LINEAR:
                if (mode == 1) {
                    output = ((timer.getTime() - startTime) / timeToTarget * (target - startReading)) + startReading;
                } else {
                    output = MyMath.bound((Math.signum(target - startReading) * (timer.getTime() - startTime) * speed) + startReading, target, startReading);
                }
                break;
            case INSTANT:
                output = target;
            default:
                output = startReading;
        }
    }

    public void setSpeed(double sped) {
        speed = sped;
    }

    public void setTimeToTarget(double timeToTarg) {
        timeToTarget = timeToTarg;
    }
}
