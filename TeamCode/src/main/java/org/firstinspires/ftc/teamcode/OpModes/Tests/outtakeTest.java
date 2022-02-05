package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.FFRobot;
import org.firstinspires.ftc.teamcode.Robots.FourWheelRobot;

@Disabled
@TeleOp(name="outtake test", group="TeleOp")
public class outtakeTest extends LinearOpMode {

    // Declare OpMode Members
    private FFRobot bot = new FFRobot(this);

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        telemetry.addData("status", "running");
        telemetry.update();

        // OUTTAKE
        double angle = 30;
        double length = 50;
        double tilt = 10;
        int x = 0;
        bot.outtake.setTargets(angle, tilt, length, 1);
        while (opModeIsActive()) {
            bot.outtake.setTargets(angle, tilt, length, 1);

            telemetry.addData("outtake state", bot.outtake.state);

            telemetry.addData("turret", bot.outtake.turretPosition);
            telemetry.addData("turret target", bot.outtake.targetTurretPosition);
            telemetry.addData("turret error", bot.outtake.turretError);

            telemetry.addData("tilt position", bot.outtake.tiltPosition);
            telemetry.addData("tilt target", bot.outtake.targetTiltPosition);
            telemetry.addData("tilt error", bot.outtake.tiltError);

            telemetry.addData("slide", bot.outtake.slidePosition);
            telemetry.addData("limit switch", bot.outtake.slideLimitSwitch);
            telemetry.addData("slide error", bot.outtake.slideError);

            if (gamepad2.a){
                bot.outtake.setTargets(0, angle,length, 1);
            }


            telemetry.update();
            bot.update();
        }

    }

    private void initialize() {
        bot.initialize(hardwareMap);
        bot.update();
        telemetry.addData("status","initialized");
        telemetry.update();
    }
}

