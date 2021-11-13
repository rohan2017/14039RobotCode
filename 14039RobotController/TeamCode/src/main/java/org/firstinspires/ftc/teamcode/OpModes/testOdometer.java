package org.firstinspires.ftc.teamcode.OpModes;

import android.os.DropBoxManager;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.MecanumChassisBot;

@Autonomous(name="Odometer Test", group="Tests")
public class testOdometer extends LinearOpMode {

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
            telemetry.addData("X", bot.odometer.x);
            telemetry.addData("Y", bot.odometer.y);
            telemetry.addData("Heading", bot.odometer.heading);
            telemetry.addData("xVel", bot.odometer.xVel);
            telemetry.addData("yVel", bot.odometer.yVel);
            telemetry.addData("HeadingVel", bot.odometer.headingVel);
            telemetry.addData("XAcceleration", bot.odometer.xAcc);
            telemetry.addData("YAcceleration", bot.odometer.yAcc);
            telemetry.update();
            bot.update();
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