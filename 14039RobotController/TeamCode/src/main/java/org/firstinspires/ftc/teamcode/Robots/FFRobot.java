package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.FFRobotHardware;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Localization.DummyOdometer;
import org.firstinspires.ftc.teamcode.Subsystems.Localization.Odometer6WD;
import org.firstinspires.ftc.teamcode.Subsystems.Movement.Drivebases.DummyTankDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Movement.MovementDummyTankDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Movement.MovementTankDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Subsystems.Timer;


public class FFRobot extends Robot {

    public DummyTankDrive drivebase;
    public MovementTankDrive movement;
    public Odometer6WD odometer;
    public Intake intake;
    public Outtake outtake;
    public Timer time;

    public FFRobot(LinearOpMode opMode) {
        super(opMode);
        this.hardware = new FFRobotHardware();
        this.drivebase = new DummyTankDrive(opMode, hardware);
        this.odometer = new Odometer6WD(opMode, hardware);
        this.movement = new MovementTankDrive(opMode, drivebase, odometer);
        this.intake = new Intake(opMode, hardware);
        this.outtake = new Outtake(opMode, hardware);
        this.time = new Timer(opMode,hardware);
    }

    public void initialize(HardwareMap hardwareMap) {
        hardware.hardwareMap(hardwareMap);
        hardware.initialize();
        odometer.initialize();
        drivebase.initialize();
        drivebase.resetDriveEncoders();
        drivebase.reverseMotors(1, -1, -1, 1);
        drivebase.setPowerBehavior("float");
        drivebase.setRunMode("frontWithEncoder");
        movement.initialize();
        //outtake.initialize();
        //intake.initialize();
        time.initialize();
    }

    public void update() {
        time.update();
        odometer.update();
        movement.update();
        drivebase.update();
        intake.update();
        //outtake.update();
    }
}
