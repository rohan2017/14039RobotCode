package org.firstinspires.ftc.teamcode.OpModes.Tests;

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
        double angle = 0;
        double length = 0;
        double tilt = 0;

        bot.outtake.setTargets(angle, tilt, length, 1);
        while (opModeIsActive()) {
            telemetry.addData("turret ticks ", bot.hardware.getMotor("turret").getCurrentPosition());
            telemetry.addData("state", bot.outtake.state);
            telemetry.addData("turret error", bot.outtake.turretError);
            telemetry.addData("turret targTicks", bot.outtake.targetTurretPosition);
            telemetry.addData("slide position ticks", bot.outtake.slidePosition);
            telemetry.addData("slide target position", bot.outtake.targetSlidePosition);
            telemetry.update();
            bot.outtake.update();
            if(gamepad1.a) {
                bot.outtake.setTargets(60, tilt, length, 1);
            }else {
                bot.outtake.setTargets(0, tilt, length, 1);
            }
        }

    }

    private void initialize() {
        bot.initialize(hardwareMap);
        bot.drivebase.setRunMode("withoutEncoder");
        telemetry.addData("status","initialized");
        telemetry.update();
    }
}

