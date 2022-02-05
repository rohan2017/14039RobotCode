package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.testBot;
import org.firstinspires.ftc.teamcode.Subsystems.State;

@Disabled
@TeleOp(name="Timer test", group="Testing")
public class timerTest extends LinearOpMode {

    // Declare OpMode Members
    private testBot bot = new testBot(this);

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        telemetry.addData("status", "running");
        telemetry.update();

        while (opModeIsActive()) {

            if(gamepad1.a) {
                bot.time.delaySeconds(5);
            }

            telemetry.addData("time", bot.time.getTime());
            telemetry.addData("time", bot.time.state.toString());
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