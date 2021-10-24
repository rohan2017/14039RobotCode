package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.MecanumChassisBot;

@Autonomous(name="Movement Test", group="Testing")
public class testMovement extends LinearOpMode {

    // Declare OpMode Members
    private MecanumChassisBot bot = new MecanumChassisBot(this, hardwareMap);

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        telemetry.addData("status","running");
        telemetry.update();

        while(opModeIsActive()) {
            bot.drivebase.setRelativeForce(0, 1, 0);
            bot.drivebase.update();
        }
        bot.drivebase.freeze();
        bot.drivebase.update();
    }

    private void initialize() {
        bot.initialize();
        telemetry.addData("status","initialized");
        telemetry.update();
    }
}