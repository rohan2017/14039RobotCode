package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.FFRobot;
import org.firstinspires.ftc.teamcode.Robots.testBot;
import org.firstinspires.ftc.teamcode.Subsystems.State;

@Disabled
@TeleOp(name="Intake test", group="Testing")
public class intakeTest extends LinearOpMode {

    // Declare OpMode Members
    private FFRobot bot = new FFRobot(this);

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        telemetry.addData("status", "running");
        telemetry.update();

        while (opModeIsActive()) {

            bot.update();
            telemetry.addData("has cargo", bot.intake.hasBlock);
            telemetry.addData("raw sensor", bot.intake.intensity);
            telemetry.addData("filtered sensor", bot.intake.filteredIntensity);
            telemetry.update();

        }
    }

    private void initialize() {
        bot.initialize(hardwareMap);
        bot.update();

        intakemode = InMode.SEARCHING;

        telemetry.addData("status","initialized");
        telemetry.update();
    }
}