package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.MathFunctions.MyMath;
import org.firstinspires.ftc.teamcode.MathFunctions.PointEx;

import java.util.ArrayList;

public class Outtake {

    private boolean here = false, here2 = false, here3 = false;

    private LinearOpMode opMode;
    private RobotHardware hardware;

    // Slide variables
    private int slidePosition = 0;
    private final double spoolDiam = 4.318;
    private final double ticksPerRevSpool = 580.4;
    private final double ticksPerCm = ticksPerRevSpool*spoolDiam*Math.PI;

    // Turret variables
    private int turretPosition = 0;
    private double gearRatioT = 33/15; // motor/turret rotations
    private double ticksPerRevTMotor = 537.6;
    private final double ticksPerDegTurret = ticksPerRevTMotor*gearRatioT/360;

    // Tilt variables
    private int tiltPosition = 0;
    private final double startAngle = 10;
    private double gearRatioP = 27/1; // motor/tilt rotation
    private double ticksPerRevPMotor = 537.6;
    private final double ticksPerDegTilt = ticksPerRevPMotor*gearRatioP/360;

    // Basket servo
    private double servoState;
    private final double receivePos = 0;
    private final double holdPos = 0.5;
    private final double dropPos = 0.7;

    public String state;

    public Outtake(LinearOpMode opMode, RobotHardware robothardware) {
        this.opMode = opMode;
        this.hardware = robothardware;
        state = "idle";
    }

    public void initialize() {
        // Turret spin motor
        hardware.getMotor("turret").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.getMotor("turret").setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.getMotor("turret").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.getMotor("turret").setDirection(DcMotor.Direction.REVERSE);
        //hardware.getMotor("turret").setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(0,0,0,0));

        // Tilt motor
        hardware.getMotor("tilt").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.getMotor("tilt").setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.getMotor("tilt").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.getMotor("tilt").setDirection(DcMotor.Direction.FORWARD);
        //hardware.getMotor("turret").setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(0,0,0,0));

        // Spool motor
        hardware.getMotor("extension").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

                if(MyMath.distance(turretPosition, slidePosition, hardware.getMotor("turret").getCurrentPosition(), hardware.getMotor("extension").getCurrentPosition()) < 10) {
                    if(Math.abs(tiltPosition - hardware.getMotor("tilt").getCurrentPosition()) < 10) {
                        state = "converged";
                    }else {
                        state = "transient";
                    }
                }// bad but works

                if(state.equals("transient")) {
                    hardware.getMotor("turret").setTargetPosition(turretPosition);
                    hardware.getMotor("tilt").setTargetPosition(tiltPosition);
                    hardware.getMotor("extension").setTargetPosition(slidePosition);
                    hardware.getMotor("turret").setPower(0.3);
                    hardware.getMotor("tilt").setPower(0.3);
                    hardware.getMotor("extension").setPower(0.3);
                    if(servoState == 0) {
                        hardware.getServo("basketFlipper").setPosition(receivePos);
                    }else if(servoState == 1) {
                        hardware.getServo("basketFlipper").setPosition(holdPos);
                    }else {
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
        if(angle > -35 && angle < 35) { // upper and lower turret angle limits
            turretPosition = (int)(angle*ticksPerDegTurret);
        }
    }

    public void setPitchAngle(double angle) {
        // 0 is straight flat, 15 is pointed up
        if(angle > 0 && angle < 25) { // upper and lower turret angle limits
            tiltPosition = (int)((angle-startAngle)*ticksPerDegTilt);
        }
    }

    // Sets the state of the dropoff
    public void setBoxState(int state) {
        // 0 - Receive
        // 1 - Hold
        // 2 - Drop
        servoState = state;
    }

    // Sets the length of the extrusion
    public void setSlideLength(double length) {
        if(length > 0 && length < 100) {
            slidePosition = (int)(length*ticksPerCm);
        }
    }

    // Sets the dropoff position relative to the bot
    public void setOuttakePosition(double x, double y, double z) {}

    public void setTargets(double turretAngle, double tiltAngle, double slideLength, int boxState) {
        setTurretAngle(turretAngle);
        setPitchAngle(tiltAngle);
        setSlideLength(slideLength);
        setBoxState(boxState);
        state = "transient";
    }

    public double getSlideLength(){
        return slidePosition;
    }
    public double getPitchAngle(){
        return tiltPosition;
    }
    public double getTurretPosition(){
        return turretPosition;
    }
    public boolean getCheckpoint1(){
        return here;
    }
    public boolean getCheckpoint2(){
        return here2;
    }
    public boolean getCheckpoint3(){
        return here3;
    }
}
