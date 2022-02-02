package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.CustomCV.Detector.Location.LEFT;
import static org.firstinspires.ftc.teamcode.CustomCV.Detector.Location.MID;
import static org.firstinspires.ftc.teamcode.CustomCV.Detector.Location.RIGHT;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.FFRobot;
import org.firstinspires.ftc.teamcode.Robots.FourWheelRobot;
import org.firstinspires.ftc.teamcode.Subsystems.State;
import org.openftc.easyopencv.OpenCvCamera;

import org.firstinspires.ftc.teamcode.CustomCV.Detector;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


@Autonomous(name="Red Auto", group="Testing")

public class redAuto extends LinearOpMode
 {
        // Declare OpMode Members
        private FFRobot bot = new FFRobot(this);
        OpenCvCamera phoneCam;
        private int turretAng, tiltAng, extrudeLength;

        @Override
        public void runOpMode() {
            initialize();
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
            Detector detector = new Detector();
            phoneCam.setPipeline(detector);

            phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    phoneCam.startStreaming(720,480, OpenCvCameraRotation.SIDEWAYS_RIGHT);
                }

                @Override
                public void onError(int errorCode) {

                }
            });
            waitForStart();

            while (opModeIsActive()) {
                if ( detector.getLocation() == LEFT){
                    turretAng= 30;
                    extrudeLength= 40;
                    tiltAng= 30;
                }
                if ( detector.getLocation() == MID){
                    turretAng= 30;
                    extrudeLength= 40;
                    tiltAng= 20;
                }
                if ( detector.getLocation() == RIGHT){
                    turretAng= 30;
                    extrudeLength= 40;
                    tiltAng= 10;
                }
            }

            sleep(1000);
            bot.outtake.setTargets(turretAng, tiltAng, extrudeLength, 1);
            sleep(2000);
            bot.outtake.setBoxState(2);
            sleep(1000);

            /*
            bot.movement.setTargets(100,0);
            bot.intake.flipDown();
            while (opModeIsActive() && !bot.movement.state.equals("converged")) {
                bot.movement.update();
            }
            bot.intake.setPower(0.8);
            */

        }


        private void initialize() {
            bot.initialize(hardwareMap);
            bot.movement.state = State.TRANSIENT;
            bot.intake.flipUp();
            bot.update();
            telemetry.addData("status","initialized");
            telemetry.update();
        }
    }

