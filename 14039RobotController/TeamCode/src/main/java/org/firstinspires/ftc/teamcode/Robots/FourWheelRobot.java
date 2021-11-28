package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.BasicRobotHardware;
import org.firstinspires.ftc.teamcode.Hardware.ChassisHardware;
import org.firstinspires.ftc.teamcode.Subsystems.Localization.DummyOdometer;
import org.firstinspires.ftc.teamcode.Subsystems.Movement.Drivebases.DummyTankDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Movement.MovementDummyTankDrive;

public class FourWheelRobot extends Robot {

    public DummyTankDrive drivebase;
    public DummyOdometer odometer;
    public MovementDummyTankDrive movement;

    public FourWheelRobot(LinearOpMode opMode) {
        super(opMode);
        hardware = new ChassisHardware();
        odometer = new DummyOdometer(opMode, hardware);
        drivebase = new DummyTankDrive(opMode, hardware);
        movement = new MovementDummyTankDrive(opMode, drivebase, odometer);
    }

    public void initialize(HardwareMap hardwareMap) {
        hardware.hardwareMap(hardwareMap);
        hardware.initialize();
        odometer.initialize();
        drivebase.initialize();
        drivebase.resetDriveEncoders();
        drivebase.reverseMotors("Right");
        drivebase.setPowerBehavior("brake");
        drivebase.setRunMode("withoutEncoder");
        movement.initialize();
    }

    public void update() {
        odometer.update();
        movement.update();
        drivebase.update();
    }
}
