package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CustomCV.RedDetector;
import org.firstinspires.ftc.teamcode.MathFunctions.PointEx;
import org.firstinspires.ftc.teamcode.Robots.FFRobot;
import org.firstinspires.ftc.teamcode.Subsystems.State;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="Fast (untested) Red Auto", group="OpMode")
public class AutoFastRed extends LinearOpMode {

    // Declare OpMode Members
    private FFRobot bot = new FFRobot(this);

    public enum autoMode {
        PRIMETRANSFER,
        ALIGNTRANSFER,
        TRANSFER,
        PRIMEHOLD,
        SETTLEHOLD,
        HOLDING,
        EXTEND,
        DEPOSIT,
        HOMESLIDE,
        HOMETURRET,
        HOMECENTER,
        HOME,
        READY
    }

    private autoMode autoM;

    OpenCvCamera phoneCam;

    private final int allianceTurret = -68;
    private final int allianceSlide = 145;
    private final int allianceTilt = 30;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        RedDetector detector = new RedDetector();
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

        int pos = 0;
        // INITIAL BARCODE DROP-OFF
        RedDetector.Location en = detector.getLocation();
        if (en == RedDetector.Location.LEFT){
            pos = 0;
        } else if (en == RedDetector.Location.MID){
            pos = 2;
        } else if (en == RedDetector.Location.RIGHT){
            pos = 1;
        }
        if(pos == 0) {
            bot.time.delaySeconds(1.5);
            bot.outtake.setTargets(-60, 6, 83, 1);
        }else if(pos == 1) {
            bot.time.delaySeconds(1.5);
            bot.outtake.setTargets(-63, 20, 90, 1); // update for different positions
        }else {
            bot.time.delaySeconds(1.5);
            bot.outtake.setTargets(allianceTurret+8, allianceTilt, allianceSlide, 1);
        }
        // Wait for extend
        while(bot.time.state != State.CONVERGED && opModeIsActive()) {
            bot.update();
        }
        // Drop off
        bot.outtake.setBoxState(2);
        bot.time.delaySeconds(0.3);
        while(bot.time.state != State.CONVERGED && opModeIsActive()) {
            bot.update();
        }
        // Retract slides
        bot.outtake.setTargets(bot.outtake.getTurretAngle(), bot.outtake.tiltPosition, 30, 1);
        // and Deploy intake
        bot.intake.setPower(0.7);
        bot.intake.setFlipPosition(0.6);
        bot.intake.setExtendPosition(0.06);
        bot.time.delaySeconds(0.4);
        while(bot.time.state != State.CONVERGED && opModeIsActive()) {
            bot.update();
        }
        // Home tilt and turret
        bot.outtake.setTargets(0, 0, 20, 1);
        bot.time.delaySeconds(0.3);
        while(bot.time.state != State.CONVERGED && opModeIsActive()) {
            bot.update();
        }
        // Home slides (no delay needed)
        while(!bot.outtake.homeSlides() && opModeIsActive()) {
            bot.update();
        }
        // Home everything to begin cycles
        bot.outtake.setTargets(0, 0, 0, 0);
        bot.time.delaySeconds(0.1);
        while(bot.time.state != State.CONVERGED && opModeIsActive()) {
            bot.update();
        }

        // CYCLING
        boolean park = false;
        double pointHeading = 0;
        autoM = autoMode.READY;
        bot.time.delaySeconds(5); // For first cycle to occur
        bot.movement.setTarget(new PointEx(0, 74, 0, 2));
        while(opModeIsActive()) {

            // Intake logic
            if(!park) {
                if(bot.odometer.y > 60 && !bot.intake.hasBlock) {
                    bot.intake.flipDown();
                    bot.intake.setPower(1);
                    if(bot.odometer.y > 70 && !bot.intake.hasBlock){
                        bot.intake.setExtendPosition(0.2 + 0.1*Math.sin(getRuntime()*2));
                    }
                }else {
                    bot.intake.flipHold();
                }
            }

            switch(autoM) {
                case PRIMETRANSFER:
                    // Extrude out intake and fLip up
                    bot.outtake.setTargets(0, 0, 0, 0);
                    bot.intake.setExtendPosition(0.08);
                    bot.intake.flipUp();

                    if (bot.time.state == State.CONVERGED) {
                        bot.time.delaySeconds(0.3); // delay is duration of the next state
                        autoM = autoMode.ALIGNTRANSFER;
                    }
                    break;
                case ALIGNTRANSFER:
                    // Retract intake and stabilize bucket
                    bot.outtake.setTargets(0, 0, 0, 0);
                    bot.outtake.setSlidePower(-0.3);
                    bot.intake.setExtendPosition(0.03);
                    bot.intake.flipUp();

                    if (bot.time.state == State.CONVERGED) {
                        bot.time.delaySeconds(0.5); // delay is duration of the next state
                        autoM = autoMode.TRANSFER;
                    }
                    break;
                case TRANSFER:
                    // Run motor
                    bot.outtake.setTargets(0, 0, 0, 0);
                    bot.outtake.setSlidePower(-0.3);
                    bot.intake.setExtendPosition(0.03);
                    bot.intake.flipUp();
                    bot.intake.setPower(-1);

                    if (bot.time.state == State.CONVERGED) {
                        bot.time.delaySeconds(0.3); // delay is duration of the next state
                        bot.intake.setPower(0);
                        autoM = autoMode.PRIMEHOLD;
                    }
                    break;
                case PRIMEHOLD:
                    // Retract slide slightly now with block
                    bot.outtake.setTargets(0, 0, 1, 0);

                    if (bot.time.state == State.CONVERGED) {
                        bot.time.delaySeconds(0.4); // delay is duration of the next state
                        autoM = autoMode.SETTLEHOLD;
                    }
                    break;
                case SETTLEHOLD:
                    // Let block settle and flip to intermediate hold
                    bot.outtake.setTargets(0, 0, 4, 3);
                    // Flip up intake to clear line
                    bot.intake.setFlipPosition(0.55);

                    if (bot.time.state == State.CONVERGED) {
                        bot.time.delaySeconds(0.3); // delay is duration of the next state
                        autoM = autoMode.HOLDING;
                    }
                    break;
                case HOLDING:
                    // Flip to hold pos and bucket past walls
                    bot.outtake.setTargets(allianceTurret, 2, 20, 1);
                    // Flip up intake to clear line
                    bot.intake.setFlipPosition(0.55);

                    if (bot.time.state == State.CONVERGED || bot.odometer.y < 5) {
                        bot.time.delaySeconds(1.8); // delay is duration of the next state
                        autoM = autoMode.EXTEND;
                    }
                    break;
                case EXTEND:
                    // Out to drop-off position
                    bot.outtake.setTargets(allianceTurret, allianceTilt, allianceSlide, 1);
                    // Flip up intake to clear line
                    bot.intake.setFlipPosition(0.55);

                    if (bot.time.state == State.CONVERGED || bot.movement.state == State.CONVERGED) {
                        bot.time.delaySeconds(0.15); // delay is duration of the next state
                        autoM = autoMode.DEPOSIT;
                    }
                    break;
                case DEPOSIT:
                    // Drop block
                    bot.outtake.setBoxState(2);
                    // Flip up intake to clear line
                    bot.intake.setFlipPosition(0.55);

                    if (bot.time.state == State.CONVERGED ) {
                        bot.time.delaySeconds(0.8); // delay is duration of the next state
                        pointHeading += 7;
                        autoM = autoMode.HOMESLIDE;
                    }
                    break;
                case HOMESLIDE:
                    // Retract slides and nothing else
                    bot.outtake.setTargets(bot.outtake.getTurretAngle(), bot.outtake.tiltPosition, 43, 1);
                    // Flip hold intake
                    bot.intake.flipHold();

                    if (bot.time.state == State.CONVERGED) {
                        bot.time.delaySeconds(0.7); // delay is duration of the next state
                        autoM = autoMode.HOMETURRET;
                    }
                    break;
                case HOMETURRET:
                    // Turret and tilt
                    bot.outtake.setTargets(0, 0, 43, 1);

                    if (bot.time.state == State.CONVERGED) {
                        bot.time.delaySeconds(0.6); // delay is duration of the next state
                        autoM = autoMode.HOMECENTER;
                    }
                    break;
                case HOMECENTER:
                    // Home slides
                    bot.outtake.setTargets(0, 0, 0, 1);

                    if (bot.time.state == State.CONVERGED) {
                        bot.time.delaySeconds(0.2); // delay is duration of the next state
                        autoM = autoMode.HOME;
                    }
                    break;
                case HOME:
                    // Home bucket
                    bot.outtake.setTargets(0, 0, 0, 0);
                    if (bot.time.state == State.CONVERGED) {
                        bot.time.delaySeconds(5);
                        autoM = autoMode.READY;
                    }
                    break;
                case READY:
                    if(bot.intake.hasBlock || bot.time.state == State.CONVERGED) {
                        bot.time.delaySeconds(0.5); // delay is duration of the next state
                        autoM = autoMode.PRIMETRANSFER;
                    }
                    break;
            }

            bot.movement.update();
            if(!park) {
                if(autoM == autoMode.DEPOSIT) {
                    bot.movement.setTarget(new PointEx(0, 74, pointHeading, 2));
                }else if(autoM == autoMode.PRIMETRANSFER) {
                    bot.movement.setTarget(new PointEx(0, -10, 0, 2));
                }else if(autoM == autoMode.EXTEND) {
                    bot.drivebase.freeze();
                }
            }

            // Emergency parking logic
            if(getRuntime() > 29 && !park) {
                park = true;
                autoM = autoMode.HOMESLIDE;
                bot.movement.setTarget(new PointEx(0, 70, 0, 2));
            }

            bot.teleUpdate(); // since we want custom movement.update()

            telemetry.addData("time", bot.time.getTime());
            telemetry.addData("mode", autoM);
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
