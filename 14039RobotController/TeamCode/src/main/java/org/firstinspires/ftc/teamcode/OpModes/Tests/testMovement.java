package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.FFRobot;
import org.firstinspires.ftc.teamcode.Robots.FourWheelRobot;
import org.firstinspires.ftc.teamcode.Robots.testBot;
import org.firstinspires.ftc.teamcode.Subsystems.State;

@Autonomous(name="Movement Test", group="Testing")
public class testMovement extends LinearOpMode {

    // Declare OpMode Members
    private FFRobot bot = new FFRobot(this);

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        bot.movement.setTargetDistance(-50);

        while (opModeIsActive() & bot.movement.state != State.CONVERGED) {
            telemetry.addData("cur distance", bot.movement.currentDistance);
            telemetry.addData("cur heading", bot.movement.currentHeading);
            telemetry.update();
            bot.update();
        }
        bot.movement.setTargetDistance(0);
        while (opModeIsActive() & bot.movement.state != State.CONVERGED) {
            telemetry.addData("cur distance", bot.movement.currentDistance);
            telemetry.addData("cur heading", bot.movement.currentHeading);
            telemetry.update();
            bot.update();
        }
        bot.movement.setTargetHeading(90);
        while (opModeIsActive() & bot.movement.state != State.CONVERGED) {
            telemetry.addData("cur distance", bot.movement.currentDistance);
            telemetry.addData("cur heading", bot.movement.currentHeading);
            telemetry.update();
            bot.update();
        }

    }

    private void initialize() {
        bot.initialize(hardwareMap);
        bot.movement.state = State.TRANSIENT;
        bot.update();
        telemetry.addData("status","initialized");
        telemetry.update();
    }
}