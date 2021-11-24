package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.BasicRobotHardware;
import org.firstinspires.ftc.teamcode.Hardware.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.Localization.Odometer2WIMU;
import org.firstinspires.ftc.teamcode.Subsystems.Movement.Drivebases.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Movement.MovementHolonomic;

public class MecanumChassisBot extends Robot {

    public MecanumDrive drivebase;
    public Odometer2WIMU odometer;
    //public Linear_Odometer odometer;
    public MovementHolonomic movement;
    public Intake intake;
    public Outtake outtake;
    public MecanumChassisBot(LinearOpMode opMode) {
        super(opMode);
        hardware = new BasicRobotHardware();
        drivebase = new MecanumDrive(opMode, hardware);
        odometer = new Odometer2WIMU(opMode, hardware);
        //odometer = new Linear_Odometer(opMode, hardware);
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
    }

    public void update() {
        drivebase.update();
        odometer.update();
        intake.update();
        movement.update();
        outtake.update();
    }
}
