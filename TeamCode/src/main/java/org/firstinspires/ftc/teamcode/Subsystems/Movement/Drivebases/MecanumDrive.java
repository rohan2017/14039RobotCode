package org.firstinspires.ftc.teamcode.Subsystems.Movement.Drivebases;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import com.qualcomm.robotcore.hardware.DcMotor;

public class MecanumDrive extends HolonomicDrivebase {

    private double lf, rf, lb, rb; //motor/wheel target forces

    private final double wheelbaseRadius = 40; //cm
    private final double robotMass = 10; //kg
    private final double rotationalInertia = robotMass * Math.pow((wheelbaseRadius/2), 2); //radius/2 assumes uniform weight distribution
    private final double wheelRadius = 5.08; //cm
    private final double gearRatio = 22.0/20.0; //motor-rpm : wheel-rpm
    private final double ticksPerRev = 10; //ticks-per-revolution of motor shaft
    // Needs to be tuned such that when multiplied by shaft torque it gives motor power.
    final double shaftTorqueToPower = 1; // Changes based on drive motors
    final double staticCoeffFriction = 0.6;
    final double kineticCoeffFriction = 0.4;
    final double forceToPowerConstant = shaftTorqueToPower * gearRatio/(wheelRadius*robotMass/4*9.81);
    final double headingAccConstant = wheelbaseRadius * Math.PI / 180; // Assuming units of deg/s
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
            hardware.getMotor("driveFrontRight").setPower(rf);
            hardware.getMotor("driveFrontLeft").setPower(lf);
            hardware.getMotor("driveBackLeft").setPower(lb);
            hardware.getMotor("driveBackRight").setPower(rb);
        }
    }

    public void setRelativeAcceleration(double accX, double accY, double accHeading) {
        /* Wheel movement to move horizontally
          A          |
          |          V
               -->
          |          A
          V          |
        */
        //https://discord.com/channels/445308068721590273/456178951849771028/891435261014192128

        double forceX = accX*robotMass;
        double forceY = accY*robotMass;

        lf = forceToPower(2*(forceY*0.9 + forceX*1.1) - (accHeading * rotationalInertia*headingAccConstant * 1.41), "lf");
        rf = forceToPower(2*(forceY*0.9 - forceX*1.1) + (accHeading * rotationalInertia*headingAccConstant * 1.41), "rf");
        lb = forceToPower(2*(forceY*0.9 - forceX*1.1) - (accHeading * rotationalInertia*headingAccConstant * 1.41), "lb");
        rb = forceToPower(2*(forceY*0.9 + forceX*1.1) + (accHeading * rotationalInertia*headingAccConstant * 1.41), "rb");

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

    private double forceToPower(double force, String wheel) {
        if(slipping(wheel)) {
            return force*forceToPowerConstant/kineticCoeffFriction;
        }else {
            return force*forceToPowerConstant/staticCoeffFriction;
        }
    }

    private boolean slipping(String wheel) {

    }

}