package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.BasicRobotHardware;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Subsystems.Localization.Odometer2WIMU;
import org.firstinspires.ftc.teamcode.Subsystems.Movement.Drivebases.MecanumDrive;

public class MecanumChassisBot extends Robot {

    public MecanumDrive drivebase;
    public Odometer2WIMU odometer;

    public MecanumChassisBot(LinearOpMode opMode) {
        super(opMode);
        hardware = new BasicRobotHardware();
        drivebase = new MecanumDrive(opMode, hardware);
        odometer = new Odometer2WIMU(opMode, hardware);
    }

    public void initialize(HardwareMap hardwareMap) {
        hardware.hardwareMap(hardwareMap);
        hardware.initialize();
        odometer.initialize();
        odometer.setEncoderDirections(-1, -1);
        drivebase.initialize();
    }

    public void update() {
        drivebase.update();
        odometer.update();
    }
}
