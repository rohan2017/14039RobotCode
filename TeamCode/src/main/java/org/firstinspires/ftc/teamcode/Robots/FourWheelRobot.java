package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CustomCV.Vision;
import org.firstinspires.ftc.teamcode.Hardware.ChassisHardware;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Localization.DummyOdometer;
import org.firstinspires.ftc.teamcode.Subsystems.Movement.Drivebases.DummyTankDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Movement.MovementDummyTankDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;

public class FourWheelRobot extends Robot {

    public DummyTankDrive drivebase;
    public DummyOdometer odometer;
    public MovementDummyTankDrive movement;

    public FourWheelRobot(LinearOpMode opMode) {
        super(opMode);
        this.hardware = new ChassisHardware();
        this.odometer = new DummyOdometer(opMode, hardware);
        this.drivebase = new DummyTankDrive(opMode, hardware);
        this.movement = new MovementDummyTankDrive(opMode, drivebase, odometer);
    }

    public void initialize(HardwareMap hardwareMap) {
        hardware.hardwareMap(hardwareMap);
        hardware.initialize();
        odometer.initialize();
        drivebase.initialize();
        drivebase.hardwareResetDriveEncoders();
        drivebase.reverseMotors("Right");
        drivebase.setPowerBehavior("float");
        drivebase.setRunMode("withEncoder");
        movement.initialize();
    }

    public void update() {
        odometer.update();
        movement.update();
        drivebase.update();
    }
}
