package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MathFunctions.PointEx;
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
        telemetry.addData("status","running");
        telemetry.update();

        bot.movement.setTarget(new PointEx(0,60,0));
        while (opModeIsActive() && bot.movement.state != State.CONVERGED) {
            telemetry.addData("movement state", bot.movement.state);
            telemetry.update();
            bot.update();
        }

        bot.movement.setTarget(new PointEx(0,0,0));
        while (opModeIsActive() && bot.movement.state != State.CONVERGED) {
            telemetry.addData("movement state", bot.movement.state);
            telemetry.update();
            bot.update();
        }

        bot.movement.setTarget(new PointEx(40,40,-15));
        while (opModeIsActive() && bot.movement.state != State.CONVERGED) {
            telemetry.addData("movement state", bot.movement.state);
            telemetry.update();
            bot.update();
        }

        bot.movement.setTarget(new PointEx(40,40,0));
        while (opModeIsActive() && bot.movement.state != State.CONVERGED) {
            telemetry.addData("movement state", bot.movement.state);
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