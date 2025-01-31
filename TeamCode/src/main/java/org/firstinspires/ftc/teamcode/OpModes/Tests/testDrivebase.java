package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.MecanumChassisBot;
import org.firstinspires.ftc.teamcode.Subsystems.State;

@TeleOp(name="Drivebase Test", group="Testing")
public class testDrivebase extends LinearOpMode {

    // Declare OpMode Members
    private MecanumChassisBot bot = new MecanumChassisBot(this);

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        telemetry.addData("status","running");
        telemetry.update();

        bot.odometer.startTracking(0,0,0);

        while(opModeIsActive()) {

            double y1 = -gamepad1.right_stick_y;
            double x1 = gamepad1.right_stick_x;
            double x2 = gamepad1.left_stick_x;
            double y2 = -gamepad1.left_stick_y;

            bot.drivebase.setPowers(y2+x2, y1-x1, y2-x2, y1+x1);
            bot.drivebase.update();
            bot.odometer.update();

            telemetry.addData("X", bot.odometer.x);
            telemetry.addData("Y",bot.odometer.y);
            telemetry.addData("H",bot.odometer.heading);
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