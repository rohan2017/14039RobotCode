package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.testBot;

@TeleOp(name="Intake test", group="Testing")
public class intakeTest extends LinearOpMode {

    // Declare OpMode Members
    private testBot bot = new testBot(this);

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        telemetry.addData("status","running");
        telemetry.update();

        while(opModeIsActive()) {

            // INTAKE
            // intake power
            if(gamepad1.right_bumper) {
                bot.intake.setPower(-0.8);
            }else if(gamepad1.right_trigger > 0.1) {
                bot.intake.setPower(0.8);
            }else {
                bot.intake.setPower(0);
            }

            // flipping
            if(gamepad1.left_stick_y < -0.1) {
                bot.intake.flipDown();
            }else if(gamepad1.left_stick_y > 0.1) {
                bot.intake.flipUp();
            }else {
                bot.intake.flipHold();
            }

            // extend
            bot.intake.setExtendPosition(gamepad1.left_trigger);

            bot.update();
            telemetry.addData("intaked cargo", bot.intake.hasBlock);
            //telemetry.addData("intensity", bot.intake.filteredIntensity);
            telemetry.update();

        }
    }

    private void initialize() {
        bot.initialize(hardwareMap);
        bot.update();
        telemetry.addData("status","initialized");
        telemetry.update();
    }
}