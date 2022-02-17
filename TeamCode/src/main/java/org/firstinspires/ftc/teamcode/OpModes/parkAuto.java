package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.OpModes.parkAuto.autoMode.DROPPOS;
import static org.firstinspires.ftc.teamcode.OpModes.parkAuto.autoMode.PRIMETRANSFER;
import static org.firstinspires.ftc.teamcode.OpModes.parkAuto.autoMode.SEARCHING;
import static org.firstinspires.ftc.teamcode.OpModes.parkAuto.autoMode.SEARCHPOS;

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
        SEARCHPOS,
        SEARCHING,
        DROPPOS,
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

    private double rightPow = 0.4, leftPow = 0.385;

    private final int allianceTurret = -68;
    private final int allianceSlide = 130;
    private final int allianceTilt = 33;

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        telemetry.addData("status","running");
        telemetry.update();

        // INITIAL BARCODE DROP-OFF
        bot.time.delaySeconds(3);
        int pos = 0;
        if(pos == 0) {
            bot.outtake.setTargets(allianceTurret, allianceSlide-40, allianceTilt, 1);
        }else if(pos == 1) {
            bot.outtake.setTargets(allianceTurret, allianceSlide-40, allianceTilt, 1); // update for different positions
        }else {
            bot.outtake.setTargets(allianceTurret, allianceSlide-40, allianceTilt, 1);
        }
        while(bot.time.state != State.CONVERGED && opModeIsActive()) {
            bot.update();
        }
        bot.time.delaySeconds(0.5);
        bot.outtake.setBoxState(2);
        while(bot.time.state != State.CONVERGED && opModeIsActive()) {
            bot.update();
        }
        bot.time.delaySeconds(0.6);
        bot.outtake.setTargets(bot.outtake.getTurretAngle(), bot.outtake.tiltPosition, 43, 1);
        while(bot.time.state != State.CONVERGED && opModeIsActive()) {
            bot.update();
        }
        bot.time.delaySeconds(0.4);
        bot.outtake.setTargets(0, 0, 43, 1);
        while(bot.time.state != State.CONVERGED && opModeIsActive()) {
            bot.update();
        }
        while(!bot.outtake.homeSlides()) {
            bot.update();
        }
        bot.time.delaySeconds(0.5);
        bot.outtake.setTargets(0, 0, 0, 0);
        while(bot.time.state != State.CONVERGED && opModeIsActive()) {
            bot.update();
        }

        // CYCLING
        autoM = autoMode.READY;
        while(opModeIsActive()) {

            // State Machine
            switch(autoM) {
                case PRIMETRANSFER:
                    bot.outtake.setTargets(0, 0, 0, 0);
                    bot.intake.setExtendPosition(0.08);
                    bot.intake.flipUp();
                    if (bot.time.state == State.CONVERGED) {
                        bot.time.delaySeconds(0.4); // delay is duration of the next state
                        autoM = autoMode.ALIGNTRANSFER;
                    }
                    break;
                case ALIGNTRANSFER:
                    bot.outtake.setTargets(0, 0, 0, 0);
                    bot.intake.setExtendPosition(0.03);
                    bot.intake.flipUp();
                    if (bot.time.state == State.CONVERGED) {
                        bot.time.delaySeconds(1); // delay is duration of the next state
                        autoM = autoMode.TRANSFER;
                    }
                    break;
                case TRANSFER:
                    bot.outtake.setTargets(0, 0, 0, 0);
                    bot.intake.setExtendPosition(0.03);
                    bot.intake.flipUp();
                    bot.intake.setPower(-1);
                    if (bot.time.state == State.CONVERGED) {
                        bot.time.delaySeconds(0.4); // delay is duration of the next state
                        autoM = autoMode.PRIMEHOLD;
                    }
                    break;
                case PRIMEHOLD:
                    bot.outtake.setTargets(0, 0, 1, 0);
                    if (bot.time.state == State.CONVERGED) {
                        bot.time.delaySeconds(0.3); // delay is duration of the next state
                        autoM = autoMode.SETTLEHOLD;
                    }
                    break;
                case SETTLEHOLD:
                    bot.outtake.setTargets(0, 0, 4, 3);
                    if (bot.time.state == State.CONVERGED) {
                        bot.time.delaySeconds(2); // delay is duration of the next state
                        autoM = autoMode.EXTEND;
                    }
                    break;
                case HOLDING:
                    bot.outtake.setTargets(0, 2, 20, 1);
                    if (bot.time.state == State.CONVERGED || bot.outtake.state == State.CONVERGED) {
                        bot.time.delaySeconds(2);
                        autoM = autoMode.EXTEND;
                    }
                    break;
                case EXTEND:
                    bot.outtake.setTargets(allianceTurret, allianceTilt, allianceSlide, 1);
                    if (bot.time.state == State.CONVERGED || bot.outtake.state == State.CONVERGED) {
                        bot.time.delaySeconds(4); // delay is duration of the next state
                        autoM = autoMode.DEPOSIT;
                    }
                    break;
                case DEPOSIT:
                    bot.outtake.setBoxState(2);
                    if (bot.time.state == State.CONVERGED || bot.outtake.state == State.CONVERGED) {
                        bot.time.delaySeconds(2); // delay is duration of the next state
                        autoM = autoMode.HOMESLIDE;
                    }
                    break;
                case HOMESLIDE:
                    bot.outtake.setTargets(bot.outtake.getTurretAngle(), bot.outtake.tiltPosition, 43, 1);
                    if (bot.time.state == State.CONVERGED) {
                        bot.time.delaySeconds(0.7); // delay is duration of the next state
                        autoM = autoMode.HOMETURRET;
                    }
                    break;
                case HOMETURRET:
                    bot.outtake.setTargets(0, 0, 43, 1);
                    if (bot.time.state == State.CONVERGED) {
                        bot.time.delaySeconds(1); // delay is duration of the next state
                        autoM = autoMode.HOMECENTER;
                    }
                    break;
                case HOMECENTER:
                    bot.outtake.setTargets(0, 0, 0, 1);
                    if (bot.time.state == State.CONVERGED) {
                        bot.time.delaySeconds(0.2); // delay is duration of the next state
                        autoM = autoMode.HOME;
                    }
                    break;
                case HOME:
                    bot.outtake.setTargets(0, 0, 0, 0);
                    if (bot.time.state == State.CONVERGED) {
                        autoM = autoMode.READY;
                    }
                    break;
                case READY:
                    bot.outtake.setBoxState(0);
                    autoM = SEARCHING;
                    break;
                case SEARCHING:
                    bot.intake.setExtendPosition(0.03);
                    bot.intake.flipDown();
                    bot.movement.update();
                    if (!bot.intake.hasBlock) {
                        bot.intake.setPower(0.8);
                        bot.drivebase.setPowers(leftPow,rightPow);
                    } else {
                        bot.drivebase.setPowers(0,0);
                        bot.movement.setTarget(new PointEx(0, 0, 0));
                        autoM = DROPPOS;
                    }
                    break;
                case DROPPOS:
                    bot.movement.update();
                    if (bot.movement.state == State.CONVERGED) {
                        autoM = PRIMETRANSFER;
                    }
                    break;
            }

            bot.teleUpdate(); // since we want custom movement.update()

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
        bot.intake.flipHold();
        bot.outtake.setTurretOffsetAngle(-90);
        bot.update();
        telemetry.addData("status","initialized");
        telemetry.update();
    }
}