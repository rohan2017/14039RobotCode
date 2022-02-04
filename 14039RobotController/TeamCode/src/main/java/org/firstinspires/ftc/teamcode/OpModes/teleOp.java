package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robots.FFRobot;
import org.firstinspires.ftc.teamcode.Robots.testBot;
import org.firstinspires.ftc.teamcode.Subsystems.State;

@TeleOp(name="AR Scrim teleOp", group="TeleOp")
public class teleOp extends LinearOpMode {

    // Declare OpMode Members
    private FFRobot bot = new FFRobot(this);

    public enum TeleMode {
        IDLE,
        INTAKE,
        PRIMETRANSFER,
        TRANSFER,
        HOLDPRIME,
        HOLDING,
        DEPOSIT,
        DUMP,
        HOMESLIDE,
        HOME,
    }

    private TeleMode teleM;
    private final int allianceTurret = -65;
    private final int allianceSlide = 140;
    private final int allianceTilt = 26;
    private boolean intakeHasBlock = false;
    private final int sharedTurret = 30;
    private final int sharedSlide = 100;
    private final int sharedTilt = 10;

    private boolean delayed = false;
    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        telemetry.addData("status","running");
        telemetry.update();

        teleM = TeleMode.INTAKE;

        while(opModeIsActive()) {
            // DRIVING
            bot.drivebase.setPowers(-Math.pow(gamepad1.left_stick_y, 3)*0.7, -Math.pow(gamepad1.right_stick_y, 3)*0.7);

            double correct = gamepad1.right_stick_y - gamepad1.left_stick_y;
            if(gamepad1.left_trigger > 0.1) {
                bot.drivebase.setPowers(-gamepad1.left_trigger + correct*0.2, -gamepad1.left_trigger - correct*0.2);
            }else if(gamepad1.right_trigger > 0.1) {
                bot.drivebase.setPowers(gamepad1.right_trigger + correct*0.2, gamepad1.right_trigger - correct*0.2);
            }

            if(gamepad2.y) {
                teleM = TeleMode.HOMESLIDE;
            }

            switch(teleM) {
                case INTAKE:
                    // INTAKE
                    delayed = false;
                    bot.outtake.setBoxState(0);
                    bot.intake.setExtendPosition((gamepad2.left_trigger + 0.35)*0.33);
                    if(gamepad2.right_bumper && !bot.intake.hasBlock) {
                        bot.intake.setPower(1);
                        bot.intake.flipDown();
                    }else if(gamepad2.left_bumper) { // && bot.outtake.readyReceive
                        bot.intake.flipUp();
                        bot.intake.setExtendPosition(0.2);
                        bot.time.delaySeconds(0.5); // WAIT FOR INTAKE TO FLIP BACK WITH INTAKE EXTENDED
                        teleM = TeleMode.PRIMETRANSFER;
                    }else {
                        bot.intake.setPower(0.1);
                        bot.intake.flipHold();
                    }
                    // OUTTAKE
                    bot.outtake.setTargets(0,0,-0.1,0);
                    break;
                case PRIMETRANSFER:
                    if(bot.time.state == State.CONVERGED) {
                        bot.intake.setExtendPosition(0.13);
                        bot.time.delaySeconds(0.4); // WAIT FOR INTAKE EXTEND TO ALIGN
                        teleM = TeleMode.TRANSFER;
                        intakeHasBlock = false;
                    }
                    break;
                case TRANSFER:
                    if(bot.time.state == State.CONVERGED) {
                        bot.outtake.setSlideLength(0.2);
                        bot.outtake.setBoxState(0);
                        bot.intake.setPower(-1);
                        // delay for handoff
                        bot.time.delaySeconds(0.5); // WAIT FOR BLOCK TO TRANSFER
                        teleM = TeleMode.HOLDPRIME;
                    }
                    break;
                case HOLDPRIME:
                    if(bot.time.state == State.CONVERGED) {
                        bot.outtake.setBoxState(3);
                        bot.time.delaySeconds(0.3);
                        teleM = TeleMode.HOLDING;
                    }
                    break;
                case HOLDING:
                    if(bot.time.state == State.CONVERGED) {
                        bot.outtake.setSlideLength(0.2);
                        bot.outtake.setBoxState(1);
                        bot.time.delaySeconds(0.1);
                        bot.intake.setPower(0);
                        bot.intake.flipHold();
                        teleM = TeleMode.DEPOSIT;
                    }
                    break;
                case DEPOSIT:
                    if(bot.time.state == State.CONVERGED) {
                        if(gamepad2.b) {
                            bot.outtake.setTargets(sharedTurret, sharedTilt, sharedSlide, 1);
                        }else if(gamepad2.a) {
                            bot.outtake.setTargets(allianceTurret, allianceTilt, 30, 1);
                            if (!delayed){
                                bot.time.delaySeconds(0.3);
                                delayed = true;
                            }
                            if (bot.time.state == State.CONVERGED) {
                                bot.outtake.setTargets(allianceTurret, allianceTilt, allianceSlide, 1);
                            }
                        }else {
                            // Manual turret
                            bot.outtake.setTurretPower(-Math.pow(gamepad2.right_stick_x, 3)*0.5);
                            // Manual slides
                            bot.outtake.incrementSlideLength(-gamepad2.left_stick_y);
                            // Manual tilt
                            if(gamepad2.dpad_up) {
                                bot.outtake.setPitchAngle(bot.outtake.tiltPosition + 3);
                            }else if(gamepad2.dpad_down) {
                                bot.outtake.setPitchAngle(bot.outtake.tiltPosition - 3);
                            }

                        }
                        // Dumping
                        if(gamepad1.right_bumper){
                            bot.outtake.setBoxState(2);
                            bot.time.delaySeconds(0.2);
                            teleM = TeleMode.DUMP;
                        }
                        break;
                    }
                case DUMP:
                    bot.intake.setExtendPosition((gamepad2.left_trigger + 0.03)*0.33);
                    if(bot.time.state == State.CONVERGED) {
                        bot.outtake.setBoxState(1);
                        if (!gamepad1.right_bumper) {
                            bot.outtake.setSlideLength(30);
                            bot.time.delaySeconds(0.4);
                            teleM = TeleMode.HOMESLIDE;
                        }
                    }
                    break;
                case HOMESLIDE:
                    bot.intake.setExtendPosition((gamepad2.left_trigger + 0.03)*0.33);
                    if(bot.time.state == State.CONVERGED || bot.outtake.slideError <= 20) {
                        bot.outtake.setTargets(0, 0, 2, 1);
                        bot.time.delaySeconds(0.2);
                        teleM = TeleMode.HOME;
                    }
                    break;
                case HOME:
                    bot.intake.setExtendPosition((gamepad2.left_trigger + 0.03)*0.33);
                    if(bot.time.state == State.CONVERGED) {
                        bot.outtake.setTargets(0, 0, 1, 0);
                        teleM = TeleMode.INTAKE;
                    }
                    break;
            }

            bot.teleUpdate();

            telemetry.addData("outtake state", bot.outtake.state);
            telemetry.addData("mode", teleM);

            telemetry.addData("turret", bot.outtake.turretPosition);
            telemetry.addData("turret target", bot.outtake.targetTurretPosition);
            telemetry.addData("turret error", bot.outtake.turretError);

            telemetry.addData("tilt position", bot.outtake.tiltPosition);
            telemetry.addData("tilt target", bot.outtake.targetTiltPosition);
            telemetry.addData("tilt error", bot.outtake.tiltError);
            telemetry.addData("tilt power", bot.outtake.tiltPower);
            telemetry.addData("extended PID  power", bot.outtake.tiltExtendedCtrl.correction);


            telemetry.addData("slide", bot.outtake.slidePosition);
            telemetry.addData("limit switch", bot.outtake.slideLimitSwitch);
            telemetry.addData("slide error", bot.outtake.slideError);
            telemetry.addData("slide PID", bot.hardware.getMotor("extension").getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).toString());

            telemetry.update();
        }
        bot.drivebase.freeze();
    }

    private void initialize() {
        bot.initialize(hardwareMap);
        bot.update();
        telemetry.addData("status","initialized");
        telemetry.update();
    }
}