package org.firstinspires.ftc.teamcode.OpModes;

import static java.lang.Math.abs;
import static java.lang.Math.log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robots.FFRobot;
import org.firstinspires.ftc.teamcode.Subsystems.State;

@TeleOp(name="New teleOp", group="TeleOp")
public class teleOpFixed extends LinearOpMode {

    // Declare OpMode Members
    private FFRobot bot = new FFRobot(this);

    public enum TMode {
        IDLE,
        PRIMETRANSFER,
        ALIGNTRANSFER,
        TRANSFER,
        PRIMEHOLD,
        SETTLEHOLD,
        HOLDING,
        DEPOSIT,
        HOMESLIDE,
        HOMETURRET,
        HOMECENTER,
        HOME,
        READY,
        EJECT
    }

    private TMode teleM;

    private final int allianceTurret = -65;
    private final int allianceSlide = 140;
    private final int allianceTilt = 26;

    private final int sharedTurret = 30;
    private final int sharedSlide = 100;
    private final int sharedTilt = 10;

    // for release behavior of right bumper drop-off
    private boolean lastGP1RB;
    private boolean GP1RB;

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        telemetry.addData("status","running");
        telemetry.update();

        teleM = TMode.HOME;

        while(opModeIsActive()) {

            // MANUAL CONTROLS ///////////////////////////////////
            // DRIVING
            bot.drivebase.setPowers(-Math.pow(gamepad1.left_stick_y, 3)*0.7, -Math.pow(gamepad1.right_stick_y, 3)*0.7);
            // Trigger assisted
            double correct = gamepad1.right_stick_y - gamepad1.left_stick_y;
            if(gamepad1.left_trigger > 0.1) {
                bot.drivebase.setPowers(0.8*(-gamepad1.left_trigger + correct*0.2), 0.8*(-gamepad1.left_trigger - correct*0.2));
            }else if(gamepad1.right_trigger > 0.1) {
                bot.drivebase.setPowers(0.8*(gamepad1.right_trigger + correct*0.2), 0.8*(gamepad1.right_trigger - correct*0.2));
            }

            // INTAKE
            bot.intake.setExtendPosition((gamepad2.left_trigger*0.33)+0.03);
            if(gamepad2.right_bumper) {
                bot.intake.setPower(1);
                bot.intake.flipDown();
            }else if(gamepad2.left_bumper && teleM == TMode.READY) {
                teleM = TMode.PRIMETRANSFER;
                bot.time.delaySeconds(0.3);
            }else {
                bot.intake.setPower(0.1);
                bot.intake.flipHold();
            }

            // OUTTAKE
            // Manual Turret
            bot.outtake.setTurretPower(-Math.pow(gamepad2.right_stick_x, 3)*0.5);
            // Manual Slides
            bot.outtake.incrementSlideLength(-gamepad2.left_stick_y);
            // Manual Tilt
            if(gamepad2.dpad_up) {
                bot.outtake.setPitchAngle(bot.outtake.targetTiltPosition + 3);
            }else if(gamepad2.dpad_down) {
                bot.outtake.setPitchAngle(bot.outtake.targetTiltPosition - 3);
            }
            // Preset Drop-off
            if(gamepad2.a) {
                bot.outtake.setTargets(allianceTurret, allianceTilt, allianceSlide, 1);
                teleM = TMode.DEPOSIT;
            }else if(gamepad2.x) {
                bot.outtake.setTargets(sharedTurret, sharedTilt, sharedSlide, 1);
                teleM = TMode.DEPOSIT;
            }
            // Dumping
            if(gamepad1.right_bumper) {
                bot.outtake.setBoxState(2);
            }
            // Y-reset
            if(gamepad2.y) {
                teleM = TMode.HOMECENTER;
            }

            // AUTOMATIC CONTROLS ///////////////////////////////////
            if(bot.intake.hasBlock && teleM == TMode.READY && bot.outtake.readyReceive) {
                teleM = TMode.PRIMETRANSFER;
                bot.time.delaySeconds(0.3);
            }
            if(gamepad2.right_trigger > 0.2 && (teleM == TMode.PRIMETRANSFER || teleM == TMode.ALIGNTRANSFER)) {
                bot.time.delaySeconds(0.4);
                teleM = TMode.EJECT;
            }
            // State Machine
            switch(teleM) {
                case PRIMETRANSFER:
                    bot.outtake.setTargets(0, 0, 0, 0);
                    bot.intake.setExtendPosition(0.08);
                    bot.intake.flipUp();
                    if(bot.time.state == State.CONVERGED) {
                        bot.time.delaySeconds(0.4); // delay is duration of the next state
                        teleM = TMode.ALIGNTRANSFER;
                    }
                    break;

                case ALIGNTRANSFER:
                    bot.outtake.setTargets(0, 0, 0, 0);
                    bot.intake.setExtendPosition(0.03);
                    bot.intake.flipUp();
                    if(bot.time.state == State.CONVERGED) {
                        bot.time.delaySeconds(1); // delay is duration of the next state
                        teleM = TMode.TRANSFER;
                    }
                    break;
                case TRANSFER:
                    bot.outtake.setTargets(0, 0, 0, 0);
                    bot.intake.setExtendPosition(0.03);
                    bot.intake.flipUp();
                    bot.intake.setPower(-1);
                    if(bot.time.state == State.CONVERGED) {
                        bot.time.delaySeconds(0.4); // delay is duration of the next state
                        teleM = TMode.PRIMEHOLD;
                    }
                    break;
                case PRIMEHOLD:
                    bot.outtake.setTargets(0, 0, 1, 0);
                    if(bot.time.state == State.CONVERGED) {
                        bot.time.delaySeconds(0.3); // delay is duration of the next state
                        teleM = TMode.SETTLEHOLD;
                    }
                    break;
                case SETTLEHOLD:
                    bot.outtake.setTargets(0, 0, 4, 3);
                    if(bot.time.state == State.CONVERGED) {
                        bot.time.delaySeconds(0.4); // delay is duration of the next state
                        teleM = TMode.HOLDING;
                    }
                    break;
                case HOLDING:
                    bot.outtake.setTargets(0, 2, 20, 1);
                    if(bot.time.state == State.CONVERGED || bot.outtake.state == State.CONVERGED) {
                        teleM = TMode.DEPOSIT;
                    }
                    break;
                case DEPOSIT:
                    GP1RB = gamepad1.right_bumper;
                    if(!GP1RB && lastGP1RB && bot.outtake.getSlideLength() > 10) {
                        bot.time.delaySeconds(0.7); // delay is duration of the next state
                        teleM = TMode.HOMESLIDE;
                    }
                    lastGP1RB = GP1RB;
                    break;
                case HOMESLIDE:
                    bot.outtake.setTargets(bot.outtake.getTurretAngle(), bot.outtake.tiltPosition, 43, 1);
                    if(bot.time.state == State.CONVERGED ) {
                        bot.time.delaySeconds(0.7); // delay is duration of the next state
                        teleM = TMode.HOMETURRET;
                    }
                    break;
                case HOMETURRET:
                    bot.outtake.setTargets(0, 0, 43, 1);
                    if(bot.time.state == State.CONVERGED) {
                        bot.time.delaySeconds(1); // delay is duration of the next state
                        teleM = TMode.HOMECENTER;
                    }
                    break;
                case HOMECENTER:
                    bot.outtake.setTargets(0, 0, 5, 1);
                    if(bot.time.state == State.CONVERGED) {
                        bot.time.delaySeconds(0.6); // delay is duration of the next state
                        teleM = TMode.HOME;
                    }
                    break;
                case HOME:
                    bot.outtake.setTargets(0, 0, 0, 0);
                    if(bot.time.state == State.CONVERGED) {
                        teleM = TMode.READY;
                    }
                    break;
                case READY:
                    bot.outtake.setBoxState(0);
                    break;
                case EJECT:
                    bot.intake.flipDown();
                    bot.intake.setPower(-1);
                    if(bot.time.state == State.CONVERGED) {
                        teleM = TMode.READY;
                    }
                    break;
            }
            if(gamepad2.right_trigger > 0.1) {
                bot.intake.setPower(-gamepad2.right_trigger);
            }

            bot.teleUpdate();

            telemetry.addData("outtake state", bot.outtake.state);
            telemetry.addData("ready receive", bot.outtake.readyReceive);
            telemetry.addData("mode", teleM);

            telemetry.addData("tilt", bot.outtake.tiltPosition);
            telemetry.update();
        }
        bot.drivebase.freeze();
    }

    private void initialize() {
        bot.initialize(hardwareMap);
        bot.update();
        lastGP1RB = false;
        GP1RB = false;
        telemetry.addData("status","initialized");
        telemetry.update();
    }
}