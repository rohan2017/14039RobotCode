package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MathFunctions.PointEx;
import org.firstinspires.ftc.teamcode.MathFunctions.TrapezoidalCurve;
import org.firstinspires.ftc.teamcode.Robots.FFRobot;
import org.firstinspires.ftc.teamcode.Robots.FourWheelRobot;
import org.firstinspires.ftc.teamcode.Robots.testBot;
import org.firstinspires.ftc.teamcode.Subsystems.Movement.PurePursuit;
import org.firstinspires.ftc.teamcode.Subsystems.State;

import java.util.ArrayList;

@Autonomous(name="PPtest Test", group="Testing")
@Disabled
public class PPtest extends LinearOpMode {

    // Declare OpMode Members
    private FFRobot bot = new FFRobot(this);
    private ArrayList<PointEx> path = new ArrayList<>();

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        telemetry.addData("status","running");
        telemetry.update();
        while (opModeIsActive()) {
            telemetry.addData("state", bot.movement.state);
            telemetry.addData("target point", bot.movement.targetPoint.toString());
            telemetry.addData("current point", bot.movement.currentPosition.toString());
            bot.movement.setTarget(path, 15);
            telemetry.update();
            bot.update();
        }
    }

    private void initialize() {
        bot.initialize(hardwareMap);
        bot.movement.state = State.TRANSIENT;
        bot.update();

        // Create paths
        path.add(new PointEx(0, 0,0, 0.7));
        path.add(new PointEx(-3, 30,0, 1));
        path.add(new PointEx(-5, 50,0, 1));
//        path.add(new PointEx(-20, 70,0, 0.4));
        //PurePursuit.generateTrajectory(path, new TrapezoidalCurve(0, 1));

        telemetry.addData("status","initialized");
        telemetry.update();
    }
}