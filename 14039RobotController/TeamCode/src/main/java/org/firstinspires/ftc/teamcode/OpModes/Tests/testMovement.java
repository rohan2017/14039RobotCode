package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Robots.FourWheelRobot;

@Autonomous(name="Movement Test", group="Testing")
public class testMovement extends LinearOpMode {

    // Declare OpMode Members
    private FourWheelRobot bot = new FourWheelRobot(this);

    private int tiltAngle=0;
    @Override
    public void runOpMode() {
        initialize();
        waitForStart();

        while (opModeIsActive()) {

            //bot.hardware.getMotor("turret").setTargetPosition(tiltAngle);
            //bot.hardware.getMotor("turret").setPower(0.4);

            telemetry.addData("turret pos",bot.hardware.getMotor("extension").getCurrentPosition());

            telemetry.update();
        }
        bot.outtake.setTargets(0, 0, 0, 0);
    }

    private void initialize() {
        bot.initialize(hardwareMap);
        bot.movement.state = "transient";
        bot.intake.flipUp();
        bot.update();
        telemetry.addData("status","initialized");
        telemetry.update();
    }
}