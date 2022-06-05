package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.BasicRobotHardware;
import org.firstinspires.ftc.teamcode.Hardware.ChassisHardware;
import org.firstinspires.ftc.teamcode.Hardware.FFRobotHardware;
import org.firstinspires.ftc.teamcode.Subsystems.Localization.Odometer2WIMU;
import org.firstinspires.ftc.teamcode.Subsystems.Movement.Drivebases.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Movement.MovementHolonomic;
import org.firstinspires.ftc.teamcode.Subsystems.Timer;

public class MecanumChassisBot extends Robot {

    public MecanumDrive drivebase;
    public Odometer2WIMU odometer;
    public MovementHolonomic movement;
    public Timer time;

    public MecanumChassisBot(LinearOpMode opMode) {
        super(opMode);
        hardware = new BasicRobotHardware();
        time = new Timer(opMode, hardware);
        drivebase = new MecanumDrive(opMode, hardware, 64, 5.08, 22.0/20);
        odometer = new Odometer2WIMU(opMode, hardware, 20, 32);
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
        drivebase.setPowerBehavior("float");
        drivebase.setRunMode("withoutEncoder");
        movement.initialize();
        time.initialize();
    }

    public void update() {
        time.update();
        odometer.update();
        movement.update();
        drivebase.update();
    }
}
