package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.FFRobotHardware;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Localization.DummyOdometer;
import org.firstinspires.ftc.teamcode.Subsystems.Movement.Drivebases.DummyTankDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Movement.MovementDummyTankDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;


public class FFRobot extends Robot {

    public DummyTankDrive drivebase;
    public MovementDummyTankDrive movement;
    public DummyOdometer odometer;
    public Intake intake;
    //public Outtake outtake;

    public FFRobot(LinearOpMode opMode) {
        super(opMode);
        hardware = new FFRobotHardware();
        drivebase = new DummyTankDrive(opMode, hardware);
        odometer = new DummyOdometer(opMode, hardware);
        movement = new MovementDummyTankDrive(opMode, drivebase, odometer);
        intake = new Intake(opMode, hardware);
        //outtake = new Outtake(opMode, hardware);
    }

    public void initialize(HardwareMap hardwareMap) {
        hardware.hardwareMap(hardwareMap);
        hardware.initialize();
        odometer.initialize();
        drivebase.initialize();
        drivebase.resetDriveEncoders();
        drivebase.reverseMotors(1, -1, -1, 1);
        drivebase.setPowerBehavior("brake");
        drivebase.setRunMode("withoutEncoder");
        movement.initialize();
        //outtake.initialize();
        intake.initialize();
    }

    public void update() {
        odometer.update();
        movement.update();
        drivebase.update();
        intake.update();
        //outtake.update();
    }
}
