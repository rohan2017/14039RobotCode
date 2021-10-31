package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.MecanumChassisBot;

@Autonomous(name="Movement Test", group="Testing")
public class testMovement extends LinearOpMode {

    // Declare OpMode Members
    private MecanumChassisBot bot = new MecanumChassisBot(this);

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        telemetry.addData("status", "running");
        telemetry.update();

        while (opModeIsActive()) {
            if (gamepad1.dpad_left) {
                bot.movement.targetX -= 0.1;
            }
            if (gamepad1.dpad_right) {
                bot.movement.targetX += 0.1;
            }
            if (gamepad1.dpad_up) {
                bot.movement.targetY += 0.1;
            }
            if (gamepad1.dpad_down) {
                bot.movement.targetY -= 0.1;
            }
            if (gamepad1.x) {
                bot.movement.targetHeading += 0.1;
            }
            if (gamepad1.b) {
                bot.movement.targetHeading -= 0.1;
            }
            if (gamepad1.left_bumper) {
                bot.movement.state = "transient";
            }else {
                bot.movement.state = "idle";
            }
            bot.movement.update();
            bot.drivebase.update();

            telemetry.addData("X", bot.odometer.x);
            telemetry.addData("Y", bot.odometer.y);
            telemetry.addData("Heading", bot.odometer.heading);
            telemetry.addData("TargetX", bot.movement.targetX);
            telemetry.addData("TargetY", bot.movement.targetY);
            telemetry.addData("TargetHeading", bot.movement.targetHeading);
            telemetry.addData("State", bot.movement.state);
            telemetry.update();
        }
    }

    private void initialize() {
        bot.initialize(hardwareMap);
        telemetry.addData("status","initialized");
        telemetry.update();
    }
}