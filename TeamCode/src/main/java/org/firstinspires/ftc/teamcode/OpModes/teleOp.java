package org.firstinspires.ftc.teamcode.OpModes;

import static java.lang.Math.abs;

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
        HOLDPRIME_1,
        HOLDPRIME_2,
        HOLDING,
        DEPOSIT,
        DUMP,
        HOME,
        Y_RESET,
        HOMING_1,
        HOMING_2,
        HOMING_3,
    }

    private TeleMode teleM;
    private TeleMode lastTeleM;

    private final int allianceTurret = -65;
    private final int allianceSlide = 140;
    private final int allianceTilt = 26;

    private final int sharedTurret = 30;
    private final int sharedSlide = 100;
    private final int sharedTilt = 10;

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        telemetry.addData("status","running");
        telemetry.update();

        teleM = TeleMode.INTAKE;
        lastTeleM = teleM;

        while(opModeIsActive()) {
            // DRIVING
            bot.drivebase.setPowers(-Math.pow(gamepad1.left_stick_y, 3)*0.7, -Math.pow(gamepad1.right_stick_y, 3)*0.7);

            double correct = gamepad1.right_stick_y - gamepad1.left_stick_y;
            if(gamepad1.left_trigger > 0.1) {
                bot.drivebase.setPowers(0.8*(-gamepad1.left_trigger + correct*0.2), 0.8*(-gamepad1.left_trigger - correct*0.2));
            }else if(gamepad1.right_trigger > 0.1) {
                bot.drivebase.setPowers(0.8*(gamepad1.right_trigger + correct*0.2), 0.8*(gamepad1.right_trigger - correct*0.2));
            }

            if(teleM == lastTeleM) {
                bot.intake.setExtendPosition((gamepad2.left_trigger + 0.35)*0.33);

                //Only do manual control if you *are* pressing a manual control button
                if(gamepad2.dpad_up || gamepad2.dpad_down || gamepad1.y || gamepad1.x ||
                abs(gamepad2.right_stick_x) > .1 || abs(gamepad2.left_stick_y) > .1 || gamepad2.right_bumper){
                    // Manual turret
                    bot.outtake.setTurretPower(-Math.pow(gamepad2.right_stick_x, 3)*0.5);
                    // Manual slides
                    bot.outtake.incrementSlideLength(-gamepad2.left_stick_y);
                    // Manual tilt
                    if(gamepad2.dpad_up) {
                        bot.outtake.setPitchAngle(bot.outtake.targetTiltPosition + 3);
                    }else if(gamepad2.dpad_down) {
                        bot.outtake.setPitchAngle(bot.outtake.targetTiltPosition - 3);
                    }
                    else if(gamepad2.x) {
                        bot.outtake.setBoxState(0);
                        teleM = TeleMode.HOLDING;
                    } else if(gamepad2.right_trigger > 0.1) {
                        bot.intake.setPower(-gamepad2.right_trigger);
                    }
                    if(gamepad2.right_bumper) {
                        bot.intake.setPower(1);
                        bot.intake.flipDown();
                    }

                }else{
                    if(gamepad2.b) {
                        //teleM = TeleMode.DEPOSIT;
                        //bot.outtake.setTargets(sharedTurret, sharedTilt, sharedSlide, 1);
                    }else if(gamepad2.a) {
                        teleM = TeleMode.DEPOSIT;
                        bot.outtake.setTargets(allianceTurret, allianceTilt, allianceSlide, 1);
                    }
                }

            }
            lastTeleM = teleM;

            if(gamepad2.y) {
                teleM = TeleMode.Y_RESET;
            }

            switch(teleM) {
                case INTAKE:
                    // INTAKE
                    bot.outtake.setBoxState(0);
                    if(gamepad2.left_bumper) { // && bot.outtake.readyReceive
                        bot.intake.flipUp();
                        bot.intake.setExtendPosition(0.2);
                        bot.time.delaySeconds(0.5); // WAIT FOR INTAKE TO FLIP BACK WITH INTAKE EXTENDED
                        teleM = TeleMode.PRIMETRANSFER;
                    }else if(gamepad2.right_bumper) {
                        // intaking
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
                    }
                    break;
                case TRANSFER:
                    if(bot.time.state == State.CONVERGED) {
                        bot.outtake.setSlideLength(0.2);
                        bot.outtake.setBoxState(0); // all the way down
                        bot.intake.setPower(-1);
                        // delay for handoff
                        bot.time.delaySeconds(0.5); // WAIT FOR BLOCK TO TRANSFER
                        teleM = TeleMode.HOLDPRIME_1;
                    }
                    break;
                case HOLDPRIME_1:
                    if(bot.time.state == State.CONVERGED) {
                        bot.intake.setPower(0);
                        bot.intake.flipUp();
                        bot.outtake.setBoxState(0);
                        bot.time.delaySeconds(0.3);
                        teleM = TeleMode.HOLDPRIME_2;
                    }
                    break;
                case HOLDPRIME_2:
                    if(bot.time.state == State.CONVERGED) {
                        bot.intake.setPower(0);
                        bot.intake.flipUp();
                        bot.outtake.setBoxState(3); //intermediate between down and holding
                        bot.time.delaySeconds(0.6);
                        teleM = TeleMode.HOLDING;
                    }
                    break;
                case HOLDING:
                    if(bot.time.state == State.CONVERGED) {
                        bot.outtake.setSlideLength(0.2);
                        bot.outtake.setBoxState(1); // holding state
                        bot.intake.setPower(0);
                        bot.intake.flipHold();
                        bot.time.delaySeconds(0.05);
                        teleM = TeleMode.DEPOSIT;
                    }
                    break;
                case DEPOSIT:
                    if(bot.time.state == State.CONVERGED) {
                        // Dumping
                        if(gamepad1.right_bumper) {
                            bot.outtake.setBoxState(2); // deposit state
                            bot.time.delaySeconds(0.7);
                            teleM = TeleMode.DUMP;
                        }
                        break;
                    }
                case DUMP: // begin pulling slides in
                    if(bot.time.state == State.CONVERGED || bot.outtake.state == State.CONVERGED) {
                        if (!gamepad1.right_bumper) {
                            // Start homing by pulling the slides in
                            bot.outtake.setTargets(bot.outtake.turretPosition,bot.outtake.tiltPosition, 50, 1);
                            bot.time.delaySeconds(0.8); //.2
                            teleM = TeleMode.HOMING_1;
                        }
                    }
                    break;
                    //HOMING SEQUENCE BEGINS HERE:
                case HOMING_1: // hold slides and rotate turret
                    if(bot.time.state == State.CONVERGED || bot.outtake.state == State.CONVERGED) {
                        if (!gamepad1.right_bumper) {
                            // Start homing by pulling the slides in
                            bot.outtake.setTargets(0,0, 50, 1);
                            bot.time.delaySeconds(0.7); //.2
                            teleM = TeleMode.HOMING_2; //Go to HOMING_2 State
                        }
                    }
                    break;
                case HOMING_2: //coming into center of robot
                    if(bot.time.state == State.CONVERGED || bot.outtake.state == State.CONVERGED) {
                        if (!gamepad1.right_bumper) {
                            // Start homing by pulling the slides in
                            bot.outtake.setTargets(0,0, 16, 1);
                            bot.time.delaySeconds(0.5);
                            teleM = TeleMode.HOMING_3; //Go HOME
                        }
                    }
                    break;

                case HOMING_3:
                    if(bot.time.state == State.CONVERGED || bot.outtake.state == State.CONVERGED) {
                        bot.outtake.setTargets(0, 0, 6, 0); // "reset" manipulator
                        bot.time.delaySeconds(0.6);
                        teleM = TeleMode.HOME;
                    }
                    break;
                case Y_RESET:
                    if(bot.time.state == State.CONVERGED || bot.outtake.state == State.CONVERGED) {
                        bot.outtake.setTargets(0, 0, 10, 0); // "reset" slides
                        bot.time.delaySeconds(0.25);
                        teleM = TeleMode.HOME;
                    }
                    break;

                case HOME:
                    if(bot.time.state == State.CONVERGED || bot.outtake.state == State.CONVERGED) {
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
            telemetry.addData("turret mode", bot.outtake.turretMode);
            telemetry.addData("turret power", bot.outtake.turretPower);

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