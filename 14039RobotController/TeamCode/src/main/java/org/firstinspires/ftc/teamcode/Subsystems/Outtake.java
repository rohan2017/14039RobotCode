package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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
    private int slideError = 0;
    private double slidePower = 1.0;
    private final double spoolDiam = 4.318;
    private final double ticksPerRevSpool = 0;
    private final double ticksPerCm = 20;
    private final int slideLimit = 60; // 1300 ticks

    // Turret variables
    public int turretMode; // 0-hold position, 1-power
    public int turretPosition = 0;
    public int targetTurretPosition = 0;
    private int turretError = 0;
    private double turretPower = 0;
    private final double gearRatioT = 33/15; // motor/turret rotations
    private final double ticksPerRevTMotor = 537.6;
    public final double ticksPerDegTurret = ticksPerRevTMotor*gearRatioT/360;
    private final double turretLimit = 60;
    private PIDF turretControl = new PIDF(0.001,0.0001,0.001,0.005,0.01,0.4,0);

    // Tilt variables
    public int tiltMode; // 0-hold position, 1-power
    public int tiltPosition = 0;
    public int targetTiltPosition = 0;
    private int tiltError = 0;
    private double tiltPower = 0;
    public double startAngle = 0; // angle it starts at
    private final double gearRatioP = 27/1; // motor/tilt rotation
    private final double ticksPerRevPMotor = 537.6;
    public final double ticksPerDegTilt = ticksPerRevPMotor*gearRatioP/360;
    private final double tiltLimit = 40;
    private PIDF tiltControl = new PIDF(0.001,0.0001,0.001,0.005,0.01,0.4,0);

    // Basket servo
    private int servoState = 0;
    private int servoError = 0;
    private final double receivePos = 0.66;
    private final double holdPos = 0.46;
    private final double dropPos = 0;

    public String state;

    public Outtake(LinearOpMode opMode, RobotHardware robothardware) {
        this.opMode = opMode;
        this.hardware = robothardware;
        state = "idle";
    }

    public void initialize() {
        // Turret spin motor
        hardware.getMotor("turret").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.getMotor("turret").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.getMotor("turret").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.getMotor("turret").setDirection(DcMotor.Direction.REVERSE);
        //hardware.getMotor("turret").setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(0,0,0,0));

        // Tilt motor
        hardware.getMotor("tilt").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.getMotor("tilt").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.getMotor("tilt").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.getMotor("tilt").setDirection(DcMotor.Direction.FORWARD);
        //hardware.getMotor("turret").setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(0,0,0,0));

        // Spool motor
        hardware.getMotor("extension").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.getMotor("extension").setTargetPosition(0);
        hardware.getMotor("extension").setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.getMotor("extension").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.getMotor("extension").setDirection(DcMotor.Direction.REVERSE);
        //hardware.getMotor("turret").setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(0,0,0,0));

        // Intake Servo
        hardware.getServo("basketFlipper").setPosition(receivePos);
        state = "transient";
    }

    public void update () {
        if(opMode.opModeIsActive()) {
            if(!state.equals("idle")) {

                tiltPosition = hardware.getMotor("tilt").getCurrentPosition();
                turretPosition = hardware.getMotor("turret").getCurrentPosition();
                slidePosition = hardware.getMotor("extension").getCurrentPosition();

                // State determination
                tiltError = Math.abs(tiltPosition - targetTiltPosition);
                turretError = Math.abs(turretPosition - targetTurretPosition);
                slideError = Math.abs(slidePosition - targetSlidePosition);
                servoError = servoState - getServoState();
                if(tiltError < 50 && turretError < 50 && slideError < 50 && servoError==0) {
                    state = "converged";
                }else {
                    state = "transient";
                }

                // Actions
                if(state.equals("transient")) {

                    // Tilt and Turret
                    turretControl.update(targetTurretPosition, turretPosition);
                    tiltControl.update(targetTiltPosition, tiltPosition);

                    if(tiltMode == 0) {
                        tiltPower = tiltControl.correction;
                        if(tiltError < 50) tiltPower = 0; // Threshold
                    }
                    if(turretMode == 0) {
                        turretPower = turretControl.correction;
                        if(turretError < 50) turretPower = 0; // Threshold
                    }

                    // Limits
                    if(turretPosition > turretLimit*ticksPerDegTurret && turretPower > 0) {turretPower = 0;}
                    if(turretPosition < -turretLimit*ticksPerDegTurret && turretPower < 0) {turretPower = 0;}

                    // Limits
                    if(tiltPosition > tiltLimit*ticksPerDegTilt && tiltPower > 0) {tiltPower = 0;}
                    if(tiltPosition < 0 && tiltPower < 0) {tiltPower = 0;} // 0 being lowest position

                    hardware.getMotor("turret").setPower(turretPower);
                    hardware.getMotor("tilt").setPower(tiltPower);

                    // Extension
                    hardware.getMotor("extension").setTargetPosition(targetSlidePosition);
                    hardware.getMotor("extension").setPower(1);

                    // Basket Servo
                    if(servoState == 0) {
                        hardware.getServo("basketFlipper").setPosition(receivePos);
                    }else if(servoState == 1) {
                        hardware.getServo("basketFlipper").setPosition(holdPos);
                    }else if(servoState == 2) {
                        hardware.getServo("basketFlipper").setPosition(dropPos);
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
            targetTiltPosition = (int)((angle-startAngle)*ticksPerDegTilt);
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
        if(thing == 1 || thing == 2 || thing == 3) servoState = thing;
    }

    public int getServoState() {
        double position = hardware.getServo("basketFlipper").getPosition();
        double dropDist = Math.abs(position - dropPos);
        double receiveDist = Math.abs(position - receivePos);
        double holdDist = Math.abs(position - holdPos);
        if(dropDist > receiveDist) {
            if(receiveDist > holdDist) {
                return 1;
            }else {return 0;}
        }else {
            if(holdDist > dropDist) {
                return 2;
            }else {
                return 1;
            }
        }
    }

    // Sets the length of the extrusion
    public void setSlideLength(double length) {
        if(length >= 0 && length <= slideLimit) {
            targetSlidePosition = (int)(length*ticksPerCm);
        }
    }

    public void setSlidePower(double power) {
        if(power < 1 && power > -1) {slidePower = power;}
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
        setTurretAngle(turretAngle);
        setPitchAngle(tiltAngle);
        setSlideLength(slideLength);
        setBoxState(boxState);
    }

}
