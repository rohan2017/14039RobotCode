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
    private int tiltAngle=0;
    @Override
    public void runOpMode() {
        initialize();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_down){
                tiltAngle--;
            } if (gamepad1.dpad_up){
                tiltAngle++;
            }

            bot.outtake.setTargets(tiltAngle,0, 0, 0);
            //bot.hardware.getMotor("turret").setTargetPosition(tiltAngle);
            //bot.hardware.getMotor("turret").setPower(0.4);

            telemetry.addData("turret pos",bot.hardware.getMotor("extension").getCurrentPosition());
            telemetry.addData("turret target pos",tiltAngle);

            bot.outtake.update();

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