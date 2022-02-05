package org.firstinspires.ftc.teamcode.OpModes;

import static java.lang.Math.abs;

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
        TRANSFER,
        PRIMEHOLD,
        HOLDING,
        DEPOSIT,
        DUMP,
        HOMESLIDE,
        INTAKEPRIME,
    }

    private TMode teleM;

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

        teleM = teleM.IDLE;

        while(opModeIsActive()) {
            // DRIVING
            bot.drivebase.setPowers(-Math.pow(gamepad1.left_stick_y, 3)*0.7, -Math.pow(gamepad1.right_stick_y, 3)*0.7);
            double correct = gamepad1.right_stick_y - gamepad1.left_stick_y;
            if(gamepad1.left_trigger > 0.1) {
                bot.drivebase.setPowers(0.8*(-gamepad1.left_trigger + correct*0.2), 0.8*(-gamepad1.left_trigger - correct*0.2));
            }else if(gamepad1.right_trigger > 0.1) {
                bot.drivebase.setPowers(0.8*(gamepad1.right_trigger + correct*0.2), 0.8*(gamepad1.right_trigger - correct*0.2));
            }

            // INTAKE
            bot.intake.setExtendPosition((gamepad2.left_trigger + 0.35)*0.33);
            if(gamepad2.right_bumper) {
                bot.intake.setPower(1);
                bot.intake.flipDown();
            }else if(gamepad2.left_bumper) {

            }else {
                bot.intake.setPower(0.1);
                bot.intake.flipHold();
            }


            bot.teleUpdate();

            telemetry.addData("outtake state", bot.outtake.state);
            telemetry.addData("mode", teleM);
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