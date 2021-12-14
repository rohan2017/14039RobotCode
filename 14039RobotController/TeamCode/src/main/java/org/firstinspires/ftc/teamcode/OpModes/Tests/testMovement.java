package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

    @Override
    public void runOpMode() {
        initialize();
        /*
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        detector detector = new detector(telemetry);
        phoneCam.setPipeline(detector);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
*/
        waitForStart();


        /*
        telemetry.addData("status", "running");
        telemetry.update();
        while (opModeIsActive()) {

            telemetry.addData("Position", detector.getLocation());
            telemetry.addData("Left Value", detector.getLeftValue());
            telemetry.addData("Mid Value", detector.getMidValue());
            telemetry.addData("Right Value", detector.getRightPercent());
            telemetry.update();

        }

        phoneCam.stopStreaming();

         */

        //bot.movement.setTargets(50, 0);
        while (opModeIsActive() ) {
            bot.intake.flipDown();
            bot.update();
            bot.intake.flipUp();
            bot.update();
            bot.intake.flipDown();
            bot.update();
            bot.intake.flipUp();
            bot.update();
            bot.intake.flipDown();
            bot.update();
            bot.intake.flipUp();
            bot.update();
            bot.intake.flipDown();
            bot.update();
            bot.intake.flipUp();
            bot.update();




        }


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