package org.firstinspires.ftc.teamcode.OpModes.Tests;

import android.os.DropBoxManager;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.FFRobot;
import org.firstinspires.ftc.teamcode.Robots.MecanumChassisBot;

@TeleOp(name="Odometer Test", group="Testing")
public class testOdometer extends LinearOpMode {

    // Declare OpMode Members
    private FFRobot bot = new FFRobot(this);

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        telemetry.addData("status","running");
        telemetry.update();
        bot.odometer.startTracking(0,0,0);

        while(opModeIsActive()) {
            bot.drivebase.setPowers(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
            bot.drivebase.update();

            telemetry.addData("X", bot.odometer.x);
            telemetry.addData("Y", bot.odometer.y);
            telemetry.addData("Heading", bot.odometer.heading);
            telemetry.update();
            bot.odometer.update();
        }
        bot.drivebase.freeze();
        bot.drivebase.update();
    }

    private void initialize() {
        bot.initialize(hardwareMap);

        telemetry.addData("status","initialized");
        telemetry.update();
    }
}