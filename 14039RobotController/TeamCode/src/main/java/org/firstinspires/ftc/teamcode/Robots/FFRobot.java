package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.FFRobotHardware;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Subsystems.Localization.Odometer2WIMU;
import org.firstinspires.ftc.teamcode.Subsystems.Movement.Drivebases.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Movement.MovementHolonomic;

public class FFRobot extends Robot {

    public MecanumDrive drivebase;
    public Odometer2WIMU odometer;
    public MovementHolonomic movement;
    public Intake intake;
    public Outtake outtake;

    public FFRobot(LinearOpMode opMode) {
        super(opMode);
        hardware = new FFRobotHardware();
        drivebase = new MecanumDrive(opMode, hardware, 40, 7.62, 22.0/20);
        odometer = new Odometer2WIMU(opMode, hardware);
        movement = new MovementHolonomic(opMode, drivebase, odometer);
        intake = new Intake(opMode, hardware);
        outtake = new Outtake(opMode, hardware);
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
        drivebase.setRunMode("withoutEncoder");
        movement.initialize();
        outtake.initialize();
        intake.initialize();
    }

    public void update() {
        odometer.update();
        movement.update();
        drivebase.update();
        intake.update();
        outtake.update();
    }
}
