package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.usb.serial.RobotUsbDeviceTty;

import org.firstinspires.ftc.teamcode.Robots.FFRobot;
import org.firstinspires.ftc.teamcode.Robots.FourWheelRobot;
import org.firstinspires.ftc.teamcode.Robots.MecanumChassisBot;

@TeleOp(name="teleOp", group="TeleOp")
public class teleOp extends LinearOpMode {

    // Declare OpMode Members
    private FourWheelRobot bot = new FourWheelRobot(this);

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        telemetry.addData("status","running");
        telemetry.update();

        // OUTTAKE
        double angle = 0;
        double length = 0;
        double tilt = 0;

        double loopCount = 0;

        while(opModeIsActive()) {

            // DRIVING
            bot.drivebase.setPowers(-gamepad1.left_stick_y, -gamepad1.right_stick_y);

            // INTAKE
            if(gamepad1.right_bumper) {
                bot.intake.setPower(-0.5);
            }else if(gamepad1.right_trigger > 0.1) {
                bot.intake.setPower(0.8);
            }else {
                bot.intake.setPower(0);
            }
            if(gamepad1.left_trigger > 0.1) {
                bot.intake.flipUp();
                if (gamepad1.left_trigger > 0.7) {
                    bot.intake.setPower(-0.8);
                }
            }else {
                bot.intake.flipDown();
            }

            // OUTTAKE
            if(gamepad2.a) {
                angle = 10;
                length = 50;
                tilt = 10;
                bot.outtake.setBoxState(1);
                loopCount = 0;
            }else if(gamepad2.b) {
                angle = 0;
                length = 0;
                tilt = 0;
            }

            // Extension manual control
            length -= gamepad2.left_stick_y;

            // Turret manual control
            if(Math.abs(gamepad2.right_stick_x) > 0.05) {
                bot.outtake.setTurretPower(-gamepad2.right_stick_x * 0.3);
                angle = (bot.outtake.turretPosition/bot.outtake.ticksPerDegTurret) - gamepad2.right_stick_x;
            }else {
                bot.outtake.setTurretAngle(angle);
                if(bot.outtake.turretMode == 1) {
                    angle = bot.outtake.turretPosition/bot.outtake.ticksPerDegTurret;
                }
            }

            // Tilt control
            if(gamepad2.dpad_up) {
                bot.outtake.setTiltPower(0.2);
                tilt = bot.outtake.tiltPosition/bot.outtake.ticksPerDegTilt + 1.3;
            }else if(gamepad2.dpad_down) {
                bot.outtake.setTiltPower(-0.2);
                tilt = bot.outtake.tiltPosition/bot.outtake.ticksPerDegTilt - 1.3;
            }else {
                if(bot.outtake.tiltMode == 1) {
                    tilt = bot.outtake.tiltPosition/bot.outtake.ticksPerDegTilt;
                }
                bot.outtake.setPitchAngle(tilt);
            }

            if(gamepad2.dpad_right) {
                bot.outtake.startAngle += 1;
            }else if(gamepad2.dpad_left) {
                bot.outtake.startAngle -= 1;
            }

            if(loopCount > 3) {
                bot.outtake.setSlideLength(length);
            }

            if(gamepad2.x) {
                bot.outtake.setBoxState(2);
            }else if(gamepad2.y) {
                bot.outtake.setBoxState(0);
            }else if(gamepad2.right_bumper) {
                bot.outtake.setBoxState(1);
            }

            bot.drivebase.update();
            bot.outtake.update();
            bot.intake.update();

            telemetry.addData("target turret", angle);
            telemetry.addData("turret", bot.outtake.turretPosition);
            telemetry.addData("turret mode", bot.outtake.turretMode);

            telemetry.addData("target tilt", tilt);
            telemetry.addData("tilt", bot.outtake.tiltPosition);
            telemetry.addData("tilt mode", bot.outtake.tiltMode);

            telemetry.addData("target length", length);
            telemetry.addData("slide", bot.outtake.slidePosition);

            telemetry.addData("servo", bot.outtake.getServoState());
            telemetry.addData("state", bot.outtake.state);
            telemetry.update();

            loopCount ++;
        }
        bot.drivebase.freeze();
    }

    private void initialize() {
        bot.initialize(hardwareMap);
        bot.drivebase.setRunMode("withoutEncoder");
        telemetry.addData("status","initialized");
        telemetry.update();
    }
}