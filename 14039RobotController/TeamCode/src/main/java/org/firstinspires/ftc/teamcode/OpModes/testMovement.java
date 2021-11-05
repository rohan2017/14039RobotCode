package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.MecanumChassisBot;
import org.firstinspires.ftc.teamcode.Subsystems.Movement.MovementHolonomic;

@Autonomous(name="Movement Test 2 ", group="Testing")
public class testMovement extends LinearOpMode {

    // Declare OpMode Members
    private MecanumChassisBot bot = new MecanumChassisBot(this);

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        telemetry.addData("status", "running");
        telemetry.update();
        while (!bot.movement.state.equals("converged")) {
            bot.movement.setTargets(20, 20, 0);
            bot.movement.update();
            bot.drivebase.update();
        }
    }

    private void initialize() {
        bot.initialize(hardwareMap);
        bot.movement.state = "trans 1ient";
        telemetry.addData("status","initialized");
        telemetry.update();
    }
}