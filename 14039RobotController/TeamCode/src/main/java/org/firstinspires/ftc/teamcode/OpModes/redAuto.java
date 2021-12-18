package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.CustomCV.detector.Location.LEFT;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.FourWheelRobot;
import org.openftc.easyopencv.OpenCvCamera;

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


@Autonomous(name="Red Auto", group="Testing")

public class redAuto extends LinearOpMode
 {
        // Declare OpMode Members
        private FourWheelRobot bot = new FourWheelRobot(this);
        OpenCvCamera phoneCam;
        private double x,y,z;

        @Override
        public void runOpMode() {
            initialize();
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
            waitForStart();


            while (opModeIsActive()) {
                if ( detector.getLocation() == LEFT){


                }
                if ( detector.getLocation() == LEFT){

                }
                if ( detector.getLocation() == LEFT){

                }



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

