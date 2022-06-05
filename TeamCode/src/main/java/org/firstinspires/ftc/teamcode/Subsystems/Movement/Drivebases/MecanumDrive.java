package org.firstinspires.ftc.teamcode.Subsystems.Movement.Drivebases;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import com.qualcomm.robotcore.hardware.DcMotor;

public class MecanumDrive extends DrivebaseHolonomic {

    private double lf, rf, lb, rb; //motor/wheel target rotational velocities

    private final double wheelbaseRadius; //cm
    private final double wheelRadius; //cm
    private final double gearRatio; //motor-rpm : wheel-rpm
    final double velToPowerConstant; // vel * this = power
    final double headingVelConstant; // Assuming units of deg/s

    private PID velocityConvergeX, velocityConvergeY, velocityConvergeH;

    public MecanumDrive(LinearOpMode opMode, RobotHardware hardware, double wheelbaseRadius, double wheelRadius, double gearRatio) {
        super(opMode, hardware);
        this.wheelbaseRadius = wheelbaseRadius;
        this.wheelRadius = wheelRadius;
        this.gearRatio = gearRatio;
        velToPowerConstant = 0.016 * gearRatio/wheelRadius;
        headingVelConstant = wheelbaseRadius * Math.PI / 180;
    }

    @Override
    public void initialize() {
        lf = 0;
        rf = 0;
        lb = 0;
        rb = 0;
        velocityConvergeX = new PID(0.00001, 0, 0.00001, 0, 0.1, 0);
        velocityConvergeY = new PID(0.00001, 0, 0.00001, 0, 0.1, 0);
        velocityConvergeH = new PID(0.00001, 0, 0.00001, 0, 0.1, 0);
    }

    @Override
    public void update() {
        if(opMode.opModeIsActive()) {

            // Ensure that no motor power is outside -1 to 1 range, preserving ratio
            double max = 0.4;
            double scaleDown = 1;
            if(Math.abs(lf) > max) {
                scaleDown = max/Math.abs(lf);
                lf *= scaleDown;
                rf *= scaleDown;
                lb *= scaleDown;
                rb *= scaleDown;
            }
            if(Math.abs(rf) > max) {
                scaleDown = max/Math.abs(rf);
                lf *= scaleDown;
                rf *= scaleDown;
                lb *= scaleDown;
                rb *= scaleDown;
            }
            if(Math.abs(lb) > max) {
                scaleDown = max/Math.abs(lb);
                lf *= scaleDown;
                rf *= scaleDown;
                lb *= scaleDown;
                rb *= scaleDown;
            }
            if(Math.abs(rb) > max) {
                scaleDown = max/Math.abs(rb);
                lf *= scaleDown;
                rf *= scaleDown;
                lb *= scaleDown;
                rb *= scaleDown;
            }

            hardware.getMotor("driveFrontRight").setPower(rf);
            hardware.getMotor("driveFrontLeft").setPower(lf);
            hardware.getMotor("driveBackLeft").setPower(lb);
            hardware.getMotor("driveBackRight").setPower(rb);
        }
    }

    public void setRelativeVelocity(double velX, double velY, double velHeading, double currentVelX, double currentVelY, double currentVelHeading) {
        /* Wheel movement to move horizontally
          A          |
          |          V
               -->
          |          A
          V          |
        */
        //https://discord.com/channels/445308068721590273/456178951849771028/891435261014192128
        velocityConvergeX.update(velX, currentVelX);
        velocityConvergeY.update(velY, currentVelY);
        velocityConvergeH.update(velHeading, currentVelHeading);

        double powX = velX*velToPowerConstant + velocityConvergeX.correction;
        double powY = velY*velToPowerConstant + velocityConvergeY.correction;
        double powH = velHeading*velToPowerConstant + velocityConvergeH.correction;

        lf = 2*(powY*0.95 + powX*1.05) - (powH *headingVelConstant *1.41);
        rf = 2*(powY*0.95 - powX*1.05) + (powH *headingVelConstant *1.41);
        lb = 2*(powY*0.95 - powX*1.05) - (powH *headingVelConstant *1.41);
        rb = 2*(powY*0.95 + powX*1.05) + (powH *headingVelConstant *1.41);
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

    public void setRunMode(String runMode) {
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

    public void resetDriveEncoders() {

        hardware.getMotor("driveFrontRight").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.getMotor("driveFrontLeft").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.getMotor("driveBackLeft").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.getMotor("driveBackRight").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

    private boolean slipping(String wheel) { // Need to figure this out. not urgent tho
        return false;
    }

    public void setPowers(double lf, double rf, double lb, double rb) {
        this.lf = lf;
        this.rf = rf;
        this.lb = lb;
        this.rb = rb;
    }

}