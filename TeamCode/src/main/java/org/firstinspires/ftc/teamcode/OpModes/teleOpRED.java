package org.firstinspires.ftc.teamcode.OpModes;

import static java.lang.Math.abs;
import static java.lang.Math.log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robots.FFRobot;
import org.firstinspires.ftc.teamcode.Subsystems.State;

@TeleOp(name="Red Teleop", group="TeleOp")
public class teleOpRED extends LinearOpMode {

    // Declare OpMode Members
    private FFRobot bot = new FFRobot(this);

    private final int allianceTurret = -68;
    private final int allianceSlide = 130;
    private final int allianceTilt = 33;

    private final int sharedTurret = 84;
    private final int sharedSlide = 67;
    private final int sharedTilt = 20;

    // for release behavior of right bumper drop-off
    private boolean lastGP1RB;
    private boolean GP1RB;
    private boolean doDaDucks = false;

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        telemetry.addData("status","running");
        telemetry.update();

        bot.botMode = FFRobot.bMode.HOME;
        bot.homeSlides();

        while(opModeIsActive()) {

            // MANUAL CONTROLS ///////////////////////////////////

            // DRIVING ----------
            bot.drivebase.setPowers(-Math.pow(gamepad1.left_stick_y, 3)*0.7, -Math.pow(gamepad1.right_stick_y, 3)*0.7);
            // Trigger assisted
            double correct = gamepad1.right_stick_y - gamepad1.left_stick_y;
            if(gamepad1.left_trigger > 0.1) {
                bot.drivebase.setPowers(0.8*(-gamepad1.left_trigger + correct*0.2), 0.8*(-gamepad1.left_trigger - correct*0.2));
            }else if(gamepad1.right_trigger > 0.1) {
                bot.drivebase.setPowers(0.8*(gamepad1.right_trigger + correct*0.2), 0.8*(gamepad1.right_trigger - correct*0.2));
            }

            // INTAKE ----------
            if(gamepad2.right_bumper) {
                bot.intake.setPower(1);
                bot.intake.flipDown();
            }else if(gamepad2.left_bumper && bot.botMode == FFRobot.bMode.READY) {
                bot.botMode = FFRobot.bMode.PRIMETRANSFER;
                bot.time.delaySeconds(0.7);
            }else {
                bot.intake.setPower(0.1);
                if(doDaDucks) {
                    bot.intake.setFlipPosition(0.7);
                }else {
                    bot.intake.flipHold();
                }
            }

            // OUTTAKE ----------
            // Manual Turret
            bot.outtake.setTurretPower(-gamepad2.right_stick_x*0.3);
            // Adjust turret offset if auto fails to home it
            if(gamepad1.dpad_right) {
                bot.outtake.turretOffset++;
            }else if(gamepad1.dpad_left) {
                bot.outtake.turretOffset--;
            }
            // Manual Slides
            bot.outtake.incrementSlideLength(-gamepad2.left_stick_y*3);
            // Manual Tilt
            if(gamepad2.dpad_up) {
                bot.outtake.setPitchAngle(bot.outtake.targetTiltPosition + 1.5);
            }else if(gamepad2.dpad_down) {
                bot.outtake.setPitchAngle(bot.outtake.targetTiltPosition - 1.5);
            }
            // Preset Drop-off
            if(gamepad2.a) {
                bot.updateDropPosition(allianceTurret, allianceTilt, allianceSlide, 2, 1.4);
                bot.botMode = FFRobot.bMode.EXTEND;
            }else if(gamepad2.x) {
                bot.updateDropPosition(sharedTurret, sharedTilt, sharedSlide, 2, 1);
                bot.botMode = FFRobot.bMode.EXTEND;
            }
            // Dumping
            if(gamepad1.right_bumper) {
                if(doDaDucks) {
                    bot.outtake.setBoxState(4);
                }else {
                    bot.outtake.setBoxState(2);
                }
            }
            // Y-reset
            if(gamepad2.y) {
                bot.botMode = FFRobot.bMode.HOMECENTER;
            }
            // Home slides
            if(gamepad1.left_stick_button && gamepad1.right_stick_button) {
                bot.homeSlides();
            }
            if (gamepad1.x){
                bot.outtake.resetTurret();
            }

            // AUTOMATIC CONTROLS //////////////////////
            if(gamepad2.right_trigger > 0.2 && (bot.botMode == FFRobot.bMode.PRIMETRANSFER)) {
                bot.time.delaySeconds(0.4);
                bot.botMode = FFRobot.bMode.EJECT;
            }

            // State machine
            GP1RB = gamepad1.right_bumper;
            bot.stateMachine(gamepad2.a || gamepad2.x, GP1RB, !GP1RB && lastGP1RB && !doDaDucks, bot.intake.hasBlock);
            lastGP1RB = GP1RB;

            // Intake reverse
            if(gamepad2.right_trigger > 0.1) {
                bot.intake.setPower(-gamepad2.right_trigger*.5);
            }

            if(gamepad2.b) {
                //setDuckPower(1);
                bot.intake.setFlipPosition(0.7);
            }else {
                //setDuckPower(0);
            }
            if(gamepad2.back) {
                doDaDucks = true;
            }else if(gamepad2.start) {
                doDaDucks = false;
            }

            bot.teleUpdate();

            telemetry.addData("outtake state", bot.outtake.state);
            telemetry.addData("ready receive", bot.outtake.readyReceive);
            telemetry.addData("mode", bot.botMode);
            telemetry.addData("intake intensity", bot.intake.intensity);
            telemetry.addData("intake filtered intensity", bot.intake.filteredIntensity);
            telemetry.addData("tilt", bot.outtake.tiltPosition);
            telemetry.addData("turret angle", bot.outtake.getTurretAngle());
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

    private void setDuckPower(double power) {
        bot.hardware.getCRServo("duckLeft").setPower(power);
        bot.hardware.getCRServo("duckRight").setPower(-power);
    }
}