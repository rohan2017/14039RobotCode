package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.FFRobot;
import org.firstinspires.ftc.teamcode.Robots.FourWheelRobot;

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
        while (opModeIsActive()) {
            bot.outtake.setTurretPower(0);

            telemetry.addData("outtake state", bot.outtake.state);
            telemetry.addData("ready receive", bot.outtake.readyReceive);

            telemetry.addData("turret angle", bot.outtake.getTurretAngle());
            telemetry.addData("turret ticks", bot.outtake.turretPosition);
            telemetry.addData("turret target", bot.outtake.targetTurretPosition);
            telemetry.addData("turret error", bot.outtake.turretError);

            telemetry.addData("tilt angle", bot.outtake.tiltPosition);
            telemetry.addData("tilt target", bot.outtake.targetTiltPosition);
            telemetry.addData("tilt error", bot.outtake.tiltError);

            telemetry.addData("slide length", bot.outtake.getSlideLength());
            telemetry.addData("slide ticks", bot.outtake.slidePosition);
            telemetry.addData("slide error", bot.outtake.slideError);

            if(gamepad2.a) {
                bot.outtake.setTargets(45, 5,40, 2);
            }else if(gamepad2.b) {
                bot.outtake.setTargets(60,16,50,2);
            }else if(gamepad2.x) {
                bot.outtake.setTargets(0, 0, 0, 0);
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

