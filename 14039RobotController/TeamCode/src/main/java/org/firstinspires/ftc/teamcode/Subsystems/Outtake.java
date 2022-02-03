package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.Controllers.PIDF;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.MathFunctions.MyMath;
import org.firstinspires.ftc.teamcode.MathFunctions.PointEx;

import java.util.ArrayList;

public class Outtake {

    private LinearOpMode opMode;
    private RobotHardware hardware;

    // Slide variables
    public int slidePosition = 0;
    public int targetSlidePosition = 0;
    public int slideError = 0;
    public boolean slideLimitSwitch = false;
    private final double spoolCircum = 26.33; // cm
    private final double ticksPerRevSpool = 145.1;
    private final double ticksPerCm = ticksPerRevSpool/spoolCircum;
    private final int slideLimit = 150; // cm

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
    private PID turretExtendedCtrl = new PID(0.008,0,0.001,0,0.5,0);
    private PID turretControl = new PID(0.005,0,0.0001,0,0.4,0);

    // Tilt variables
    public int tiltMode; // 0-hold position, 1-power
    public double tiltPosition = 0;
    public double targetTiltPosition = 0;
    public double tiltError = 0;
    private double tiltPower = 0;
    private final double tiltLimit = 18.5; // degrees
    private PIDF tiltExtendedCtrl = new PIDF(0.06,0.0002,0,0.001,0.08,0.5,0);
    private PIDF tiltControl = new PIDF(0.04,0.0001,0,0.001,0.07,0.4,0);

    // Basket servo
    private int servoState = 0;
    private int servoError = 0;
    private final double receivePos = 0.11;
    private final double primePos = 0.3;
    private final double holdPos = 0.5;
    private final double dropPos = 0.99;

    public State state;

    public Outtake(LinearOpMode opMode, RobotHardware robothardware) {
        this.opMode = opMode;
        this.hardware = robothardware;
        state = State.TRANSIENT;
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
        hardware.getMotor("extension").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.getMotor("extension").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.getMotor("extension").setDirection(DcMotor.Direction.REVERSE);
        //hardware.getMotor("extension").setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(0,0,0,0));

        // RESET SLIDES
        /*
        while(!slideLimitSwitch) {
            slideLimitSwitch = !hardware.getDigitalInput("slideLimit").getState();
            hardware.getMotor("extension").setPower(-0.2);
        }
        hardware.getMotor("extension").setPower(0);
         */
        hardware.getMotor("extension").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.getMotor("extension").setTargetPosition(0);
        hardware.getMotor("extension").setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Intake Servo
        hardware.getServo("basketFlipper").setPosition(receivePos);
        state = State.TRANSIENT;
    }

    public void update () {
        if(opMode.opModeIsActive()) {
            if(state != State.IDLE) {

                slideLimitSwitch = !hardware.getDigitalInput("slideLimit").getState();
                tiltPosition = getTiltAngle();
                turretPosition = hardware.getMotor("turret").getCurrentPosition();
                slidePosition = hardware.getMotor("extension").getCurrentPosition();

                // State determination
                tiltError = Math.abs(tiltPosition - targetTiltPosition);
                turretError = Math.abs(turretPosition - targetTurretPosition);
                slideError = Math.abs(slidePosition - targetSlidePosition);

                if(tiltError < 1 && turretError < 8 && slideError < 10 && servoError == 0) {
                    state = State.CONVERGED;
                }else {
                    state = State.TRANSIENT;
                }

                // Actions
                if(state == State.TRANSIENT) {

                    // Tilt and Turret
                    turretControl.update(targetTurretPosition, turretPosition);
                    turretExtendedCtrl.update(targetTurretPosition, turretPosition);
                    tiltControl.update(targetTiltPosition, tiltPosition);

                    if(tiltMode == 0) {
                        tiltPower = tiltControl.correction;//+ pivotAngleHoldPower();
                        if(tiltError < 0.3) tiltPower = 0; // Threshold
                    }
                    if(turretMode == 0) {
                        if(slidePosition > 500) {
                            turretPower = turretExtendedCtrl.correction;
                        }else {
                            turretPower = turretControl.correction;
                        }
                        //if(turretError < 20) turretPower = 0; // Threshold
                    }

                    // Limits
                    if(turretPosition > turretLimit*ticksPerDegTurret && turretPower > 0) {turretPower = 0;}
                    if(turretPosition < -turretLimit*ticksPerDegTurret && turretPower < 0) {turretPower = 0;}

                    // Limits
                    if(tiltPosition > tiltLimit && tiltPower > 0) {tiltPower = 0;}
                    if(tiltPosition < -2 && tiltPower < 0) {tiltPower = 0;} // 0 being lowest position

                    hardware.getMotor("turret").setPower(turretPower);
                    hardware.getMotor("tilt").setPower(tiltPower);

                    // Extension
                    hardware.getMotor("extension").setTargetPosition(targetSlidePosition);
                    if(slideError < 30 && slidePosition < 20) {
                        hardware.getMotor("extension").setPower((double)slideError/100);
                    }else {
                        hardware.getMotor("extension").setPower(0.8);
                    }


                    // Basket Servo
                    if(servoState == 0) {
                        hardware.getServo("basketFlipper").setPosition(receivePos);
                        servoError = 0;
                    }else if(servoState == 1) {
                        hardware.getServo("basketFlipper").setPosition(holdPos);
                        servoError = 0;
                    }else if(servoState == 2) {
                        hardware.getServo("basketFlipper").setPosition(dropPos);
                        servoError = 0;
                    }else if(servoState == 3) {
                        hardware.getServo("basketFlipper").setPosition(primePos);
                        servoError = 0;
                    }
                }
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
        if(thing == 0 || thing == 1 || thing == 2 || thing == 3) {
            servoError = thing - servoError;
            servoState = thing;
        }

    }

    // Sets the length of the extrusion
    public void setSlideLength(double length) {
        if(length <= -1) length = -1;
        if(length >= slideLimit) length = slideLimit;
        targetSlidePosition = (int)(length*ticksPerCm);
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
        return (hardware.getAnalogInput("slidePivot").getVoltage()/3.3 - 0.51)*270;
    }

}
