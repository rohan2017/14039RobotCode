package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.BasicRobotHardware;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Subsystems.Localization.Odometer2WIMU;
import org.firstinspires.ftc.teamcode.Subsystems.Movement.Drivebases.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Movement.MovementHolonomic;

public class MecanumChassisBot extends Robot {

    public MecanumDrive drivebase;
    public Odometer2WIMU odometer;
    public MovementHolonomic movement;

    public MecanumChassisBot(LinearOpMode opMode) {
        super(opMode);
        hardware = new BasicRobotHardware();
        drivebase = new MecanumDrive(opMode, hardware);
        odometer = new Odometer2WIMU(opMode, hardware);
        movement = new MovementHolonomic(opMode, drivebase, odometer);
    }

    public void initialize(HardwareMap hardwareMap) {
        hardware.hardwareMap(hardwareMap);
        hardware.initialize();
        odometer.initialize();
        odometer.setEncoderDirections(-1, -1);
        drivebase.initialize();
        drivebase.resetDriveEncoders();
        drivebase.reverseMotors("Right");
        drivebase.setPowerBehavior("brake");
        drivebase.setRunMode("withEncoder");
        movement.initialize();
    }

    public void update() {
        drivebase.update();
        odometer.update();
    }
}
