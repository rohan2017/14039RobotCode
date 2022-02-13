package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Controllers.GatedPID;
import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.Controllers.PIDF;
import org.firstinspires.ftc.teamcode.Controllers.TurretController;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.MathFunctions.MyMath;
import org.firstinspires.ftc.teamcode.MathFunctions.PointEx;

import java.util.ArrayList;

public class Outtake {

    private LinearOpMode opMode;
    private RobotHardware hardware;

    // Slide variables
    private double slidePower;
    public int slidePosition = 0;
    public int targetSlidePosition = 0;
    public int slideError = 0;
    //public boolean slideLimitSwitch = false;
    private final double spoolCircum = 26.33; // cm
    private final double ticksPerRevSpool = 145.1;
    private final double ticksPerCm = ticksPerRevSpool/spoolCircum;
    private final int slideLimit = 160; // cm
    private PIDF slideExtendedCtrl = new PIDF(0.003, 0.00001, 0.0003, 0.002, 0.05, 0.8, 0);
    private PIDF slideNearCtrl = new PIDF(0.008, 0.00001, 0.0002, 0.002, 0.08, 0.8, 0);

    // Turret variables
    public int turretMode; // 0-hold position, 1-power
    public int turretPosition = 0;
    public int targetTurretPosition = 0;
    public int turretError = 0;
    public double turretPower = 0;
    private final double gearRatioT = 10/3; // motor/turret rotations
    private final double ticksPerRevTMotor = 751.8;
    public final double ticksPerDegTurret = ticksPerRevTMotor*gearRatioT/360;
    private final double turretLimit = 80;
    private TurretController turretControl = new TurretController(slideLimit);

    // Tilt variables
    public int tiltMode; // 0-hold position, 1-power
    public double tiltPosition = 0;
    public double targetTiltPosition = 0;
    public double tiltError = 0;
    public double tiltPower = 0;
    private final double tiltLimit = 30; // degrees
    public PIDF tiltExtendedCtrl = new PIDF(0.11,0.0000,0,0,0,1,0);
    public PIDF tiltControl = new PIDF(0.035,0.0001,0,0.001,0.07,0.4,0);

    // Basket servo
    private int servoState = 0;
    private int servoError = 0;
    private final double receivePos = 0.02;//SAVOX CAN NOT BE SET TO ZERO!
    private final double primePos = 0.13;
    private final double holdPos = 0.23;
    private final double dropPos = 0.95;

    public State state;
    public boolean readyReceive = false;

    public Outtake(LinearOpMode opMode, RobotHardware robothardware) {
        this.opMode = opMode;
        this.hardware = robothardware;
    }

    public void initialize() {
        // Turret spin motor
        hardware.getMotor("turret").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.getMotor("turret").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.getMotor("turret").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.getMotor("turret").setDirection(DcMotor.Direction.REVERSE);

        // Tilt motor
        hardware.getMotor("tilt").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.getMotor("tilt").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.getMotor("tilt").setDirection(DcMotor.Direction.REVERSE);

        // Spool motor
        hardware.getMotor("extension").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.getMotor("extension").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.getMotor("extension").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.getMotor("extension").setDirection(DcMotor.Direction.REVERSE);

        // Intake Servo
        hardware.getServo("basketFlipper").setPosition(receivePos);
        state = State.TRANSIENT;
    }

    public void update () {
        if(opMode.opModeIsActive()) {
            if(state != State.IDLE) {

                //slideLimitSwitch = !hardware.getDigitalInput("slideLimit").getState();
                tiltPosition = getTiltAngle();
                turretPosition = hardware.getMotor("turret").getCurrentPosition();
                slidePosition = hardware.getMotor("extension").getCurrentPosition();

                // State determination
                tiltError = Math.abs(tiltPosition - targetTiltPosition);
                turretError = Math.abs(turretPosition - targetTurretPosition);
                slideError = Math.abs(slidePosition - targetSlidePosition);

                if(tiltError < 1 && turretError < 20 && slideError < 10 && servoError == 0) {
                    state = State.CONVERGED;
                }else {
                    state = State.TRANSIENT;
                }

                readyReceive = (getSlideLength() < 2 && Math.abs(getTurretAngle()) < 8 && tiltPosition < 5 && servoState == 0);

                // Actions
                // Tilt and Turret
                turretControl.update(targetTurretPosition, turretPosition, slidePosition/ticksPerCm);
                tiltControl.update(targetTiltPosition, tiltPosition);
                tiltExtendedCtrl.update(targetTiltPosition,tiltPosition);
                slideExtendedCtrl.update(targetSlidePosition, slidePosition);
                slideNearCtrl.update(targetSlidePosition, slidePosition);

                if(slidePosition > 500) {
                    slidePower = slideExtendedCtrl.correction;
                } else{
                    slidePower = slideNearCtrl.correction;
                }

                if(tiltMode == 0) {
                    if(slidePosition > 500) {
                        tiltPower = tiltExtendedCtrl.correction;
                    } else{
                        tiltPower = tiltControl.correction;
                    }
                }

                if(turretMode == 0) {
                    turretPower = turretControl.correction;
                }

                // Turret Limits
                if(turretPosition > (turretLimit*ticksPerDegTurret) && turretPower > 0) {turretPower = 0;}
                if(turretPosition < -(turretLimit*ticksPerDegTurret) && turretPower < 0) {turretPower = 0;}

                // Tilt Limits
                if(tiltPosition > tiltLimit && tiltPower > 0) {tiltPower = 0;}
                if(tiltPosition < -2 && tiltPower < 0) {tiltPower = 0;} // 0 being lowest position

                // Slide Limits
                if(slidePosition > slideLimit*ticksPerCm && slidePower > 0) {slidePower = 0;}
                if(slidePosition < -5 && slidePower < 0) {slidePower = 0;}

                hardware.getMotor("turret").setPower(turretPower);
                hardware.getMotor("tilt").setPower(tiltPower);
                hardware.getMotor("extension").setPower(slidePower);

                // Basket Servo
                if(servoState == 0) {
                    hardware.getServo("basketFlipper").setPosition(receivePos);
                }else if(servoState == 1) {
                    hardware.getServo("basketFlipper").setPosition(holdPos);
                }else if(servoState == 2) {
                    hardware.getServo("basketFlipper").setPosition(dropPos);
                }else if(servoState == 3) {
                    hardware.getServo("basketFlipper").setPosition(primePos);
                }
                servoError = 0;
            }else {
                hardware.getMotor("extension").setPower(0);
                hardware.getMotor("tilt").setPower(0);
                hardware.getMotor("turret").setPower(0);
                hardware.getServo("basketFlipper").setPosition(receivePos);
            }
        }
    }

    public void setTurretAngle(double angle) {
        // 0 is straight/center, positive is counter-clockwise rotation
        if(angle > -turretLimit && angle < turretLimit) { // upper and lower turret angle limits
            targetTurretPosition = (int)(angle*ticksPerDegTurret);
            turretMode = 0;
        }
    }

    public double getTurretAngle() {
        return (double)turretPosition/ticksPerDegTurret;
    }

    public void setTurretPower(double power) {
        if(power < 1 && power > -1) {
            turretPower = power;
            turretMode = 1;
        }
    }

    public void setPitchAngle(double angle) {
        // 0 is straight flat, 15 is pointed up
        if(angle >= 0 && angle < tiltLimit) { // upper and lower turret angle limits
            targetTiltPosition = angle;
            tiltMode = 0;
        }
    }

    public void setTiltPower(double power) {
        if(power < 1 && power > -1) {
            tiltPower = power;
            tiltMode = 1;
        }
    }

    // Sets the state of the dropoff
    public void setBoxState(int thing) {
        // 0 - Receive
        // 1 - Hold
        // 2 - Drop
        // 3 - Between Recieve and Hold
        if(thing == 0 || thing == 1 || thing == 2 || thing == 3) {
            if(thing == servoState) {
                servoError = 0;
            }else {
                servoError = 1;
            }
            servoState = thing;
        }
    }

    // Sets the length of the extrusion
    public void setSlideLength(double length) {
        if(length <= -1) length = -0.25;
        if(length >= slideLimit) length = slideLimit;
        targetSlidePosition = (int)(length*ticksPerCm);
    }

    public double getSlideLength() {
        return (double)slidePosition/ticksPerCm;
    }

    public void incrementSlideLength(double increment) {
        double length = (double)targetSlidePosition/ticksPerCm + increment;
        setSlideLength(length);
    }

    // Sets the dropoff position relative to the bot
    public void setOuttakePosition(double x, double y, double z) {
        double flatLength = Math.hypot(x,y);
        double length = Math.hypot(flatLength, z);
        double pitch = Math.toDegrees(Math.asin(z/length));
        double angle = Math.toDegrees(Math.atan2(y, x)) - 90;
        setSlideLength(length);
        setPitchAngle(pitch);
        setTurretAngle(angle);
    }

    public void setTargets(double turretAngle, double tiltAngle, double slideLength, int boxState) {
        state = State.TRANSIENT;
        setTurretAngle(turretAngle);
        setPitchAngle(tiltAngle);
        setSlideLength(slideLength);
        setBoxState(boxState);
    }

    private double pivotAngleHoldPower() { // Determine FF constant for holding slides level
        return 0.05*slidePosition*ticksPerCm/slideLimit;
    }

    private double getTiltAngle() {
        return (hardware.getAnalogInput("slidePivot").getVoltage()/3.3 - 0.51)*270 + 2; // offset for some reason
    }

}
