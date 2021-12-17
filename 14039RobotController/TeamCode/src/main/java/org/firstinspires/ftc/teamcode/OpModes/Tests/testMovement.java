package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CustomCV.detector;
import org.firstinspires.ftc.teamcode.Robots.FourWheelRobot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="Movement Test", group="Testing")
public class testMovement extends LinearOpMode {

    // Declare OpMode Members
    private FourWheelRobot bot = new FourWheelRobot(this);
    OpenCvCamera phoneCam;
    private double tiltAngle;
    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        //bot.movement.setTargets(50, 0);


        while (opModeIsActive()) {
            if (gamepad1.dpad_down){
                tiltAngle--;
            } if (gamepad1.dpad_up){
                tiltAngle++;
            }
            bot.outtake.setTargets(gamepad1.left_stick_x*40, tiltAngle, gamepad1.right_stick_y*100, 0);

            telemetry.addData("left stick x scaled", gamepad1.left_stick_x*100);
            telemetry.addData("tilt angle", tiltAngle);
            telemetry.addData("extrusion length scaled",gamepad1.right_stick_y*150 );

            telemetry.addData("turret pos", bot.outtake.getTurretPosition());
            telemetry.addData("tilt pos", bot.outtake.getPitchAngle());
            telemetry.addData("slide pos", bot.outtake.getSlideLength());
            bot.update();

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