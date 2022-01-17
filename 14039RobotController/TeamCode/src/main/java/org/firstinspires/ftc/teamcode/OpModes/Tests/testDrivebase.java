package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.FFRobot;
import org.firstinspires.ftc.teamcode.Subsystems.State;

@TeleOp(name="Drivebase Test", group="Testing")
public class testDrivebase extends LinearOpMode {

    // Declare OpMode Members
    private FFRobot bot = new FFRobot(this);

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        telemetry.addData("status","running");
        telemetry.update();

        while(opModeIsActive()) {

            double y1 = -gamepad1.right_stick_y;
            double x1 = gamepad1.right_stick_x;
            double x2 = gamepad1.left_stick_x;
            double y2 = -gamepad1.left_stick_y;

            //bot.drivebase.setPowers((y2+x2), (y1-x1), (y2-x2), (y1+x1));
            bot.drivebase.setPowers(y2, y1);

            bot.drivebase.update();

            telemetry.update();
        }
        bot.drivebase.freeze();
    }

    private void initialize() {
        bot.initialize(hardwareMap);
        bot.movement.state = State.TRANSIENT;
        telemetry.addData("status","initialized");
        telemetry.update();
    }
}