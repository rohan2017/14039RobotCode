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
    private HardwareMap hardwareMap;

    public MecanumChassisBot(LinearOpMode opMode, HardwareMap hardwareMap) {
        super(opMode);
        hardware = new BasicRobotHardware();
        this.hardwareMap = hardwareMap;
        drivebase = new MecanumDrive(opMode, hardware);
        odometer = new Odometer2WIMU(opMode, hardware);
    }

    public void initialize() {
        hardware.hardwareMap(hardwareMap);
        odometer.initialize();
        drivebase.initialize();
    }
}
