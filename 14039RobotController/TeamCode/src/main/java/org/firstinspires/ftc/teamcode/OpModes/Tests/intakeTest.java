package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.testBot;
import org.firstinspires.ftc.teamcode.Subsystems.State;

@TeleOp(name="Intake test", group="Testing")
public class intakeTest extends LinearOpMode {

    public enum InMode {
        IDLE,
        SEARCHING,
        FOUND,
        PRIMED,
        DUMPED
    }

    // Declare OpMode Members
    private testBot bot = new testBot(this);
    private boolean startCycle = false;
    private double extendLength = 0.3;

    InMode intakemode;

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        telemetry.addData("status", "running");
        telemetry.update();

        while (opModeIsActive()) {

            // INTAKE
            /*
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
            */

            if(intakemode == InMode.SEARCHING) {
                bot.intake.flipDown();
                bot.intake.setExtendPosition(extendLength + 0.1*Math.sin(getRuntime()*2));
                bot.intake.setPower(0.8);
                if(bot.intake.hasBlock) {
                    intakemode = InMode.FOUND;
                    bot.time.delaySeconds(1);
                }
            }else if(intakemode == InMode.FOUND) {
                bot.intake.flipHold();
                bot.intake.setPower(0.3);
                bot.intake.retract();
                if(bot.time.state == State.CONVERGED) {
                    intakemode = InMode.PRIMED;
                    bot.time.delay(500);
                }
            }else if(intakemode == InMode.PRIMED) {
                bot.intake.flipUp();
                if(bot.time.state == State.CONVERGED) {
                    intakemode = InMode.DUMPED;
                    bot.time.delay(500);
                }
            }else if(intakemode == InMode.DUMPED) {
                bot.intake.setPower(-0.8);
                if(bot.time.state == State.CONVERGED) {
                    intakemode = InMode.SEARCHING;
                }
            }
            bot.update();
            telemetry.addData("intaked cargo", bot.intake.hasBlock);
            telemetry.addData("intake state", intakemode);
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