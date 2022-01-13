package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class Timer {

    private LinearOpMode opMode;
    private RobotHardware hardware;

    private double targetTime;
    private double time;
    public State state;

    public Timer(LinearOpMode opMode, RobotHardware robothardware) {
        this.opMode = opMode;
        this.hardware = robothardware;
    }

    public void initialize() {
        hardware.resetTimer();
        targetTime = 0;
        time = 0;
        state = State.CONVERGED;
    }

    public void update() {
        time = hardware.getTime();
        if(time < targetTime) {
            state = State.TRANSIENT;
        }else {
            state = State.CONVERGED;
        }
    }

    public void delay(int millis) {
        targetTime = millis + time;
    }

    public void delaySeconds(int seconds) {
        targetTime = seconds*1000 + time;
    }

    public void setTargetTime(int targTime) {
        targetTime = targTime;
    }

    public double getTime() {
        return time;
    }

}
