package org.firstinspires.ftc.teamcode.Subsystems.Movement.Drivebases;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class DummyTankDrive extends Drivebase {

    private double leftPower, rightPower;

    private double leftCal, rightCal;

    private double wheelRadius;
    private double gearRatio; // motor rotations per wheel rotations
    private double TpR;
    public double wheelBase;
    private double ticksPerCm;

    public DummyTankDrive(LinearOpMode opMode, RobotHardware hardware) {
        super(opMode, hardware);
        setDriveConstants(5.07, 1, 537.6, 24.35);
    }

    public void setDriveConstants(double wheelRadius, double gearRatio, double TpR, double wheelBase) {
        this.wheelRadius = wheelRadius;
        this.gearRatio = gearRatio;
        this.TpR = TpR;
        this.wheelBase = wheelBase;
        ticksPerCm = TpR*gearRatio/(wheelRadius*2*Math.PI);
    }

    @Override
    public void initialize(){
        leftPower = 0;
        rightPower = 0;
        rightCal = 0;
        leftCal = 0;
    }

    @Override
    public void update(){
        if(opMode.opModeIsActive()){
            hardware.getMotor("driveFrontRight").setPower(rightPower);
            hardware.getMotor("driveFrontLeft").setPower(leftPower);
            hardware.getMotor("driveBackLeft").setPower(leftPower);
            hardware.getMotor("driveBackRight").setPower(rightPower);
        }
    }

    public void update(double leftCorrect, double rightCorrect) {
        if(opMode.opModeIsActive()){
            hardware.getMotor("driveFrontRight").setPower(rightPower + rightCorrect);
            hardware.getMotor("driveFrontLeft").setPower(leftPower + leftCorrect);
            hardware.getMotor("driveBackLeft").setPower(leftPower + leftCorrect);
            hardware.getMotor("driveBackRight").setPower(rightPower + rightCorrect);
        }
    }

    @Override
    public void freeze(){

        leftPower = 0;
        rightPower = 0;
        update();

    }

    public void setPowerBehavior(String behavior){

        if(behavior.equals("brake")){
            hardware.getMotor("driveFrontRight").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hardware.getMotor("driveFrontLeft").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hardware.getMotor("driveBackLeft").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hardware.getMotor("driveBackRight").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }else if(behavior.equals("float")){
            hardware.getMotor("driveFrontRight").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            hardware.getMotor("driveFrontLeft").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            hardware.getMotor("driveBackLeft").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            hardware.getMotor("driveBackRight").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public void setRunMode(String runMode){

        if(runMode.equals("withEncoder")){
            hardware.getMotor("driveFrontRight").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardware.getMotor("driveFrontLeft").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardware.getMotor("driveBackLeft").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardware.getMotor("driveBackRight").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }else if(runMode.equals("withoutEncoder")){
            hardware.getMotor("driveFrontRight").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hardware.getMotor("driveFrontLeft").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hardware.getMotor("driveBackLeft").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hardware.getMotor("driveBackRight").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }else if(runMode.equals("frontWithEncoder")) {
            hardware.getMotor("driveFrontRight").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardware.getMotor("driveFrontLeft").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardware.getMotor("driveBackLeft").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hardware.getMotor("driveBackRight").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void hardwareResetDriveEncoders() {

        hardware.getMotor("driveFrontRight").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.getMotor("driveFrontLeft").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.getMotor("driveBackLeft").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.getMotor("driveBackRight").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void resetDriveEncoders() {
        leftCal = getLeftEncoder();
        rightCal = getRightEncoder();
    }

    public void reverseMotors(String side) {

        // Reverse the necessary motors so that when positive power is set to all four, the robot moves forward
        if(side.equals("Right")) {
            hardware.getMotor("driveFrontRight").setDirection(DcMotor.Direction.REVERSE);
            hardware.getMotor("driveFrontLeft").setDirection(DcMotor.Direction.FORWARD);
            hardware.getMotor("driveBackLeft").setDirection(DcMotor.Direction.FORWARD);
            hardware.getMotor("driveBackRight").setDirection(DcMotor.Direction.REVERSE);
        }else if(side.equals("Left")) {
            hardware.getMotor("driveFrontRight").setDirection(DcMotor.Direction.FORWARD);
            hardware.getMotor("driveFrontLeft").setDirection(DcMotor.Direction.REVERSE);
            hardware.getMotor("driveBackLeft").setDirection(DcMotor.Direction.REVERSE);
            hardware.getMotor("driveBackRight").setDirection(DcMotor.Direction.FORWARD);
        }

    }

    public void reverseMotors(int dirFR, int dirFL, int dirBL, int dirBR) {
        if(dirFR == 1) {
            hardware.getMotor("driveFrontRight").setDirection(DcMotor.Direction.FORWARD);
        }else {
            hardware.getMotor("driveFrontRight").setDirection(DcMotor.Direction.REVERSE);
        }
        if(dirFL == 1) {
            hardware.getMotor("driveFrontLeft").setDirection(DcMotor.Direction.FORWARD);
        }else {
            hardware.getMotor("driveFrontLeft").setDirection(DcMotor.Direction.REVERSE);
        }
        if(dirBL == 1) {
            hardware.getMotor("driveBackLeft").setDirection(DcMotor.Direction.FORWARD);
        }else {
            hardware.getMotor("driveBackLeft").setDirection(DcMotor.Direction.REVERSE);
        }
        if(dirBR == 1) {
            hardware.getMotor("driveBackRight").setDirection(DcMotor.Direction.FORWARD);
        }else {
            hardware.getMotor("driveBackRight").setDirection(DcMotor.Direction.REVERSE);
        }
    }

    public double getRightEncoder() {
        return (hardware.getMotor("driveFrontRight").getCurrentPosition() + hardware.getMotor("driveBackRight").getCurrentPosition())/(2.0*ticksPerCm) - rightCal;
    }

    public double getLeftEncoder() {
        return (hardware.getMotor("driveFrontLeft").getCurrentPosition() + hardware.getMotor("driveBackLeft").getCurrentPosition())/(2.0*ticksPerCm) - leftCal;
    }

    public void setPowers(double leftPower1, double rightPower1) {
        leftPower = leftPower1;
        rightPower = rightPower1;
    }

}
