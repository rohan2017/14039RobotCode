package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CustomCV.Vision;
import org.firstinspires.ftc.teamcode.Hardware.ChassisHardware;
import org.firstinspires.ftc.teamcode.Hardware.testHardware;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Localization.DummyOdometer;
import org.firstinspires.ftc.teamcode.Subsystems.Movement.Drivebases.DummyTankDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Movement.Movement;
import org.firstinspires.ftc.teamcode.Subsystems.Movement.MovementDummyTankDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.Timer;

public class testBot extends Robot {

    public Intake intake;
    public Timer time;
    public DummyTankDrive drivebase;
    public DummyOdometer odometer;
    public MovementDummyTankDrive movement;

    public testBot(LinearOpMode opMode) {
        super(opMode);
        this.hardware = new testHardware();
        //this.intake = new Intake(opMode, hardware);
        this.time = new Timer(opMode,hardware);
        this.odometer = new DummyOdometer(opMode, hardware);
        this.drivebase = new DummyTankDrive(opMode, hardware);
        this.movement = new MovementDummyTankDrive(opMode, drivebase, odometer);    }

    public void initialize(HardwareMap hardwareMap) {
        hardware.hardwareMap(hardwareMap);
        hardware.initialize();
        //intake.initialize();
        time.initialize();
        odometer.initialize();
        drivebase.initialize();
        drivebase.hardwareResetDriveEncoders();
        drivebase.reverseMotors("Right");
        drivebase.setPowerBehavior("float");
        drivebase.setRunMode("withEncoder");
        movement.initialize();
    }

    public void update() {
        //intake.update();
        time.update();
        odometer.update();
        movement.update();
        drivebase.update();
    }
}
