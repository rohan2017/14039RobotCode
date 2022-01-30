package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.FFRobot;
import org.firstinspires.ftc.teamcode.Robots.testBot;
import org.firstinspires.ftc.teamcode.Subsystems.State;

@TeleOp(name="teleOp", group="TeleOp")
public class teleOp extends LinearOpMode {

    // Declare OpMode Members
    private FFRobot bot = new FFRobot(this);

    public enum TeleMode {
        IDLE,
        INTAKE,
        TRANSFER,
        HOLDING,
        DEPOSIT,
        DUMP,
        HOME,
    }

    private TeleMode teleM;
    private final int allianceTurret = -60;
    private final int allianceSlide = 140;
    private final int allianceTilt = 18;

    private final int sharedTurret = 30;
    private final int sharedSlide = 100;
    private final int sharedTilt = 10;


    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        telemetry.addData("status","running");
        telemetry.update();

        double length = 0;
        teleM = TeleMode.INTAKE;

        while(opModeIsActive()) {

            // DRIVING
            bot.drivebase.setPowers(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
            if(gamepad1.left_trigger > 0.1) {
                bot.drivebase.setPowers(-gamepad1.left_trigger, -gamepad1.left_trigger);
            }else if(gamepad1.right_trigger > 0.1) {
                bot.drivebase.setPowers(gamepad1.right_trigger, gamepad1.right_trigger);
            }
            // intake extend
            bot.intake.setExtendPosition((gamepad2.left_trigger + 0.03)*0.33);

            switch(teleM) {
                case INTAKE:
                    // INTAKE
                    if(gamepad2.right_bumper && !bot.intake.hasBlock) {
                        bot.intake.setPower(1);
                        bot.intake.flipDown();
                    }else if(gamepad2.left_bumper) { // && bot.outtake.readyReceive
                        bot.intake.flipUp();
                        bot.intake.setExtendPosition(0);
                        bot.time.delaySeconds(0.5);
                        teleM = TeleMode.TRANSFER;
                    }else {
                        bot.intake.setPower(0);
                        bot.intake.flipHold();
                    }
                    // OUTTAKE
                    bot.outtake.setTargets(0,0,0,0);
                    bot.hardware.getMotor("extension").setPower(-0.1);
                    break;
                case TRANSFER:
                    if(bot.time.state == State.CONVERGED) {
                        bot.intake.setPower(-1);
                        bot.time.delaySeconds(0.3);
                        teleM = TeleMode.HOLDING;
                    }
                    break;
                case HOLDING:
                    if(bot.time.state == State.CONVERGED) {
                        bot.outtake.setBoxState(1);
                        bot.time.delaySeconds(0.1);
                        //Delay of 0.1 seconds to give the handoff some time to occur
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
                            bot.outtake.setTargets(allianceTurret, allianceTilt, allianceSlide, 1);
                        }
                        // Manual turret
                        double turPow = gamepad2.right_stick_x;
                        if(turPow > 0.1) {
                            if(Math.abs(turPow - 0.1) < 0.1) turPow = 0.1;
                            bot.outtake.setTurretPower(turPow - 0.1);
                        }else if(turPow < -0.1) {
                            if(Math.abs(turPow + 0.1) < 0.1) turPow = -0.1;
                            bot.outtake.setTurretPower(turPow + 0.1);
                        }
                        // Manual slides
                        //length -= gamepad2.left_stick_y;
                        //bot.outtake.setSlideLength(length);
                        if(gamepad1.left_bumper){
                            teleM = TeleMode.DUMP;
                        }
                        break;
                    }
                case DUMP:
                    if(bot.time.state == State.CONVERGED) {
                        bot.outtake.setBoxState(2);
                        if (!gamepad1.left_bumper) {
                            bot.outtake.setTargets(0, 0, 0, 0);
                            bot.time.delaySeconds(0.5);
                            teleM = TeleMode.INTAKE;
                        }
                    }
                    break;

            }

            bot.teleUpdate();
            telemetry.addData("drive powers", gamepad1.left_stick_y);

            telemetry.addData("receive state", bot.outtake.readyReceive);
            telemetry.addData("turret", bot.outtake.turretPosition);
            telemetry.addData("turret mode", bot.outtake.turretMode);

            telemetry.addData("tilt position", bot.outtake.tiltPosition);
            telemetry.addData("tilt mode", bot.outtake.tiltMode);

            telemetry.addData("slide", bot.outtake.slidePosition);
            telemetry.addData("target slide", bot.outtake.targetSlidePosition);

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