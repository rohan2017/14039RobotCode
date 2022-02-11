package org.firstinspires.ftc.teamcode.Controllers;

/*
This is a pretty self-explanatory PID controller. sumLimit is the limit of the error sum, and thus constrains the I-term.
 */

public class PIDF extends Controller {

    private double pGain, iGain, dGain, fGain;
    private double errorSum, sumLimit, lastError, errorSlope;
    private double P, I, D, F, correctUpLimit, correctLowLimit;
    private double lastSetPoint;
    private boolean firstLoop = true;

    // PID constructor: kP, kI, kD, limit of errorSum, limit of correction (all limits are positive and constrained about 0)
    public PIDF(double pGain, double iGain, double dGain, double fGain, double iLimit, double correctUpLimit, double correctLowLimit) {

        this.pGain = pGain;
        this.iGain = iGain;
        this.dGain = dGain;
        this.fGain = fGain;
        if(iGain != 0) {
            this.sumLimit = iLimit/iGain;
        }else {
            this.sumLimit = 0;
        }
        this.correctUpLimit = correctUpLimit;
        this.correctLowLimit = correctLowLimit;

    }

    @Override
    public void update(double setPoint, double current){

        error = setPoint - current;

        errorSum += error;
        if(errorSum > sumLimit){
            errorSum = sumLimit;
        }else if(errorSum < -sumLimit){
            errorSum = -sumLimit;
        }

        if(firstLoop){
            firstLoop = false;
            errorSlope = 0;
            lastSetPoint = setPoint;
        }else{
            errorSlope = error - lastError;
        }

        P = error * pGain;
        I = errorSum * iGain;
        D = errorSlope * dGain;
        F = (setPoint - lastSetPoint) * fGain;

        correction = P + I + D + F;

        if(correction > correctUpLimit){
            correction = correctUpLimit;
        }else if(correction < -correctUpLimit){
            correction = -correctUpLimit;
        }else if(correction > 0 && correction < correctLowLimit){
            correction = correctLowLimit;
        }else if(correction < 0 && correction > -correctLowLimit){
            correction = -correctLowLimit;
        }

        lastError = error;
        lastSetPoint = setPoint;

    }

    public void updateGains(double kP, double kI, double kD, double kF, double iLim) {
        this.pGain = kP;
        this.iGain = kI;
        this.dGain = kD;
        this.fGain = kF;
        if(iGain != 0) {
            this.sumLimit = iLim/iGain;
        }else {
            this.sumLimit = 0;
        }
    }

}