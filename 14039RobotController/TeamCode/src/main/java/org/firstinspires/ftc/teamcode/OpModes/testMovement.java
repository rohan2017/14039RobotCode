package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.MecanumChassisBot;
import org.firstinspires.ftc.teamcode.Subsystems.Movement.MovementHolonomic;

@Autonomous(name="Movement Test", group="Tests")
public class testMovement extends LinearOpMode {

    // Declare OpMode Members
    private MecanumChassisBot bot = new MecanumChassisBot(this);

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        telemetry.addData("status", "running");
        telemetry.update();
        bot.movement.setTargets(0, 80, 0);
        while (opModeIsActive() && !bot.movement.state.equals("converged")) {
            bot.movement.update();
            bot.drivebase.update();
            telemetry.addData("distance", bot.movement.getDistance());
            telemetry.addData("speed", bot.movement.getSpeed());
            telemetry.addData("total distance", bot.movement.getTotalDistance());
            telemetry.update();
        }
    }

    private void initialize() {
        bot.initialize(hardwareMap);
        bot.movement.state = "transient";
        telemetry.addData("status","initialized");
        telemetry.update();
    }
}