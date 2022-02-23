package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CustomCV.BlueDetector;
import org.firstinspires.ftc.teamcode.MathFunctions.PointEx;
import org.firstinspires.ftc.teamcode.Robots.FFRobot;
import org.firstinspires.ftc.teamcode.Subsystems.State;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="Blue Side Auto", group="OpMode")
public class AutoBlue extends LinearOpMode {

    // Declare OpMode Members
    private FFRobot bot = new FFRobot(this);
    private OpenCvCamera phoneCam;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        BlueDetector detector = new BlueDetector();
        phoneCam.setPipeline(detector);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        initialize();
        waitForStart();
        telemetry.addData("status","running");
        telemetry.update();
        bot.time.initialize();

        // INITIAL BARCODE DROP-OFF
        BlueDetector.Location en = detector.getLocation();
        if (en == BlueDetector.Location.LEFT){
            bot.time.delaySeconds(1.5);
            bot.outtake.setTargets(60, 6, 83, 1);
        } else if (en == BlueDetector.Location.MID){
            bot.time.delaySeconds(1.5);
            bot.outtake.setTargets(63, 20, 90, 1); // update for different positions
        } else if (en == BlueDetector.Location.RIGHT){
            bot.time.delaySeconds(1.5);
            bot.outtake.setTargets(63, 30, 100, 1);
        }
        // Wait for extend
        // and Deploy intake
        bot.intake.setPower(1);
        bot.intake.setFlipPosition(0.7);
        bot.intake.setExtendPosition(0.09);
        while(bot.time.state != State.CONVERGED || bot.outtake.state == State.CONVERGED && opModeIsActive()) {
            bot.update();
        }
        // Set mode to deposit to enter state machine
        bot.botMode = FFRobot.bMode.DEPOSIT;

        // CYCLING
        boolean park = false;
        double pointHeading = 0;
        while(opModeIsActive()) {

            // Intake logic
            if(!park) {
                if(bot.odometer.y > 73 && !bot.intake.hasBlock) {
                    bot.intake.flipDown();
                    bot.intake.setPower(1);
                }else {
                    bot.intake.flipHold();
                    bot.intake.setPower(0.1);
                }
            }

            bot.stateMachine(true, bot.movement.state == State.CONVERGED, true, bot.intake.hasBlock || bot.time.state == State.CONVERGED);
            bot.updateDropPosition(-68, 30, 120, 1.4);

            bot.movement.update();
            if(!park) {
                if(bot.botMode == FFRobot.bMode.DEPOSIT) {
                    bot.movement.setTarget(new PointEx(0, 80, pointHeading, 2));
                }else if(bot.botMode == FFRobot.bMode.PRIMETRANSFER) {
                    bot.movement.setTarget(new PointEx(0, -10, 0, 1.3));
                }else if((bot.botMode == FFRobot.bMode.READY || bot.botMode == FFRobot.bMode.HOME || bot.botMode == FFRobot.bMode.HOMECENTER) && !bot.intake.hasBlock && bot.odometer.y > 73) {
                    bot.drivebase.setPowers(0.25, 0.26);
                }
            }

            // Emergency parking logic
            if(getRuntime() > 28.9 && !park) {
                park = true;
                bot.botMode = FFRobot.bMode.HOMESLIDE;
                bot.time.delaySeconds(0.3);
                bot.intake.setFlipPosition(0.7);
                bot.intake.setExtendPosition(0.01);
                bot.movement.setTarget(new PointEx(0, 88, 0, 2));
            }
            // Prevent the bot from cycling after park
            if(park && bot.botMode == FFRobot.bMode.PRIMETRANSFER) {
                bot.botMode = FFRobot.bMode.READY;
            }

            bot.teleUpdate(); // since we want custom movement.update()

            telemetry.addData("mode", bot.botMode);
            telemetry.addData("has block?", bot.intake.hasBlock);
            telemetry.addData("movement target", bot.movement.targetPoint.toString());
            telemetry.addData("park?", park);

            telemetry.update();
        }
        bot.drivebase.freeze();
    }

    private void initialize() {
        bot.initialize(hardwareMap);
        bot.outtake.setTurretOffsetAngle(77.2);
        bot.update();
        telemetry.addData("status","initialized");
        telemetry.update();
    }
}
