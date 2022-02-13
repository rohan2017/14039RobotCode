package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.OpModes.parkAuto.autoMode.PRIMETRANSFER;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MathFunctions.PointEx;
import org.firstinspires.ftc.teamcode.Robots.FFRobot;
import org.firstinspires.ftc.teamcode.Robots.FourWheelRobot;
import org.firstinspires.ftc.teamcode.Robots.testBot;
import org.firstinspires.ftc.teamcode.Subsystems.State;

@Autonomous(name="Park Auto", group="OpMode")
public class parkAuto extends LinearOpMode {


    // Declare OpMode Members
    private FFRobot bot = new FFRobot(this);

    public enum autoMode {
        SEARCHING,
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
        READY,
    }

    private autoMode autoM;


    private final int allianceTurret = -65;
    private final int allianceSlide = 140;
    private final int allianceTilt = 26;

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        telemetry.addData("status","running");
        telemetry.update();

        autoM = autoMode.EXTEND;
        bot.time.delaySeconds(4);
        while(opModeIsActive()) {

//
//            // INTAKE
//            bot.intake.setExtendPosition((gamepad2.left_trigger*0.33)+0.03);
//            if(gamepad2.right_bumper) {
//                bot.intake.setPower(1);
//                bot.intake.flipDown();
//            }else if(gamepad2.left_bumper && autoM == autoMode.READY) {
//                autoM = autoMode.PRIMETRANSFER;
//                bot.time.delaySeconds(0.3);
//            }else {
//                bot.intake.setPower(0.1);
//                bot.intake.flipHold();
//            }
//            // AUTOMATIC CONTROLS ///////////////////////////////////
//            if(bot.intake.hasBlock && autoM == autoMode.READY && bot.outtake.readyReceive) {
//                autoM = autoMode.PRIMETRANSFER;
//                bot.time.delaySeconds(0.3);
//            }

            // State Machine
            switch(autoM) {
                case PRIMETRANSFER:
                    bot.outtake.setTargets(0, 0, 0, 0);
                    bot.intake.setExtendPosition(0.08);
                    bot.intake.flipUp();
                    if(bot.time.state == State.CONVERGED) {
                        bot.time.delaySeconds(0.4); // delay is duration of the next state
                        autoM =autoMode.ALIGNTRANSFER;
                    }
                    break;

                case ALIGNTRANSFER:
                    bot.outtake.setTargets(0, 0, 0, 0);
                    bot.intake.setExtendPosition(0.03);
                    bot.intake.flipUp();
                    if(bot.time.state == State.CONVERGED) {
                        bot.time.delaySeconds(1); // delay is duration of the next state
                        autoM = autoMode.TRANSFER;
                    }
                    break;
                case TRANSFER:
                    bot.outtake.setTargets(0, 0, 0, 0);
                    bot.intake.setExtendPosition(0.03);
                    bot.intake.flipUp();
                    bot.intake.setPower(-1);
                    if(bot.time.state == State.CONVERGED) {
                        bot.time.delaySeconds(0.4); // delay is duration of the next state
                        autoM = autoMode.PRIMEHOLD;
                    }
                    break;
                case PRIMEHOLD:
                    bot.outtake.setTargets(0, 0, 1, 0);
                    if(bot.time.state == State.CONVERGED) {
                        bot.time.delaySeconds(0.3); // delay is duration of the next state
                        autoM = autoMode.SETTLEHOLD;
                    }
                    break;
                case SETTLEHOLD:
                    bot.outtake.setTargets(0, 0, 4, 3);
                    if(bot.time.state == State.CONVERGED) {
                        bot.time.delaySeconds(4); // delay is duration of the next state
                        autoM = autoMode.EXTEND;
                    }
                    break;
                case HOLDING:
                    bot.outtake.setTargets(0, 2, 20, 1);
                    if(bot.time.state == State.CONVERGED || bot.outtake.state == State.CONVERGED) {
                        autoM = autoMode.EXTEND;
                    }
                    break;

                case EXTEND:
                    bot.outtake.setTargets(allianceTurret, allianceTilt, allianceSlide, 1);
                    if(bot.time.state == State.CONVERGED || bot.outtake.state == State.CONVERGED) {
                        bot.time.delaySeconds(4); // delay is duration of the next state
                        autoM = autoMode.DEPOSIT;
                    }
                    break;

                case DEPOSIT:
                    bot.outtake.setBoxState(2);
                    if(bot.time.state == State.CONVERGED || bot.outtake.state == State.CONVERGED) {
                        bot.time.delaySeconds(4); // delay is duration of the next state
                        autoM = autoMode.HOMESLIDE;
                    }
                    break;

                case HOMESLIDE:
                    bot.outtake.setTargets(bot.outtake.getTurretAngle(), bot.outtake.tiltPosition, 43, 1);
                    if(bot.time.state == State.CONVERGED ) {
                        bot.time.delaySeconds(0.7); // delay is duration of the next state
                        autoM = autoMode.HOMETURRET;
                    }
                    break;
                case HOMETURRET:
                    bot.outtake.setTargets(0, 0, 43, 1);
                    if(bot.time.state == State.CONVERGED) {
                        bot.time.delaySeconds(1); // delay is duration of the next state
                        autoM = autoMode.HOMECENTER;
                    }
                    break;
                case HOMECENTER:
                    bot.outtake.setTargets(0, 0, 5, 1);
                    if(bot.time.state == State.CONVERGED) {
                        bot.time.delaySeconds(0.6); // delay is duration of the next state
                        autoM = autoMode.HOME;
                    }
                    break;
                case HOME:
                    bot.outtake.setTargets(0, 0, 0, 0);
                    if(bot.time.state == State.CONVERGED) {
                        autoM = autoMode.READY;
                    }
                    break;
                case READY:
                    bot.outtake.setBoxState(0);
            }


            bot.teleUpdate();

            telemetry.addData("outtake state", bot.outtake.state);
            telemetry.addData("ready receive", bot.outtake.readyReceive);
            telemetry.addData("mode", autoM);

            telemetry.addData("tilt", bot.outtake.tiltPosition);
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