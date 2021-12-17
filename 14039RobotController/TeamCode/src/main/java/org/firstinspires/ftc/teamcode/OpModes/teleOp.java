package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.FFRobot;
import org.firstinspires.ftc.teamcode.Robots.FourWheelRobot;
import org.firstinspires.ftc.teamcode.Robots.MecanumChassisBot;

@TeleOp(name="teleOp", group="TeleOp")
public class teleOp extends LinearOpMode {

    // Declare OpMode Members
    private FourWheelRobot bot = new FourWheelRobot(this);

    private int turretPosition = 0;
    private double gearRatioP = 27/1; // motor/tilt rotation
    private double ticksPerRevPMotor = 537.6;
    private final double ticksPerDegTilt = ticksPerRevPMotor*gearRatioP/360;
    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        telemetry.addData("status","running");
        telemetry.update();

        while(opModeIsActive()) {
            bot.hardware.getMotor("tilt").setTargetPosition((int)(5*ticksPerDegTilt));
            bot.hardware.getMotor("tilt").setPower(-0.2);


            telemetry.update();
        }
        bot.drivebase.freeze();
    }

    private void initialize() {
        bot.initialize(hardwareMap);
        telemetry.addData("status","initialized");
        telemetry.update();
    }
}