package org.firstinspires.ftc.teamcode.Subsystems.Movement.Drivebases;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import com.qualcomm.robotcore.hardware.DcMotor;

public class MecanumDrive extends Drivebase {

    private double lf, rf, lb, rb; //motor target rotations-per-second

    private final double wheelbaseWidth = 40; //cm
    private final double wheelDiameter = 10.16; //cm
    private final double gearRatio = 22.0/20.0; //motor-rpm : wheel-rpm
    private final double ticksPerRev = 10; //ticks-per-revolution of motor shaft

    private String zeroPowerBehavior;
    private String runMode;

    public MecanumDrive(LinearOpMode opMode, RobotHardware hardware) {
        super(opMode, hardware);
    }

    @Override
    public void initialize() {
        lf = 0;
        rf = 0;
        lb = 0;
        rb = 0;
    }

    @Override
    public void update() {
        if(opMode.opModeIsActive()) {
            if(runMode.equals("withEncoder")) {
                hardware.getMotor("driveFrontRight").setVelocity(rf /ticksPerRev);
                hardware.getMotor("driveFrontLeft").setVelocity(lf / ticksPerRev);
                hardware.getMotor("driveBackLeft").setVelocity(lb / ticksPerRev);
                hardware.getMotor("driveBackRight").setVelocity(rb / ticksPerRev);
            }else if(runMode.equals("withoutEncoder")) {
                // Doesn't work until conversion is tuned
                hardware.getMotor("driveFrontRight").setPower(rpsToPower(rf));
                hardware.getMotor("driveFrontLeft").setPower(rpsToPower(lf));
                hardware.getMotor("driveBackLeft").setPower(rpsToPower(lb));
                hardware.getMotor("driveBackRight").setPower(rpsToPower(rb));
            }
        }
    }

    public void setRelativeVelocity(double velX, double velY, double velHeading) {
        /* Wheel movement to move horizontally
          A          |
          |          V
               -->
          |          A
          V          |
        */
        //https://discord.com/channels/445308068721590273/456178951849771028/891435261014192128

        lf = velY + velX*1.3 - velHeading;
        rf = velY - velX*1.3 + velHeading;
        lb = velY - velX*1.3 - velHeading;
        rb = velY + velX*1.3 + velHeading;

    }

    @Override
    public void freeze() {

        lf = 0;
        rf = 0;
        lb = 0;
        rb = 0;
        update();

    }

    public void setPowerBehavior(String behavior) {
        this.zeroPowerBehavior = behavior;
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

    private void setRunMode(String runMode) {
        this.runMode = runMode;
        if(runMode.equals("withEncoder")) {
            hardware.getMotor("driveFrontRight").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardware.getMotor("driveFrontLeft").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardware.getMotor("driveBackLeft").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardware.getMotor("driveBackRight").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }else if(runMode.equals("withoutEncoder")) {
            hardware.getMotor("driveFrontRight").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hardware.getMotor("driveFrontLeft").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hardware.getMotor("driveBackLeft").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hardware.getMotor("driveBackRight").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private void resetDriveEncoders() {

        hardware.getMotor("driveFrontRight").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.getMotor("driveFrontLeft").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.getMotor("driveBackLeft").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.getMotor("driveBackRight").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    private void reverseMotors() {

        // Reverse the necessary motors so that when positive power is set to all four, the robot moves forward
        hardware.getMotor("driveFrontRight").setDirection(DcMotor.Direction.REVERSE);
        hardware.getMotor("driveFrontLeft").setDirection(DcMotor.Direction.FORWARD);
        hardware.getMotor("driveBackLeft").setDirection(DcMotor.Direction.FORWARD);
        hardware.getMotor("driveBackRight").setDirection(DcMotor.Direction.REVERSE);

    }

    public void testMotorDirections() { //Robot moves forwards

        lf = 1;
        rf = 1;
        lb = 1;
        rb = 1;

    }

    private double rpsToPower(double rps) {
        return rps*0.7; //Idk gotta tune this value, might not even be linear
    }

}
