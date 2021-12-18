package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
        boolean reset = true;

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
                angle = -40;
                length = 45;
                tilt = 20;
                bot.outtake.setBoxState(1);
                loopCount = 0;
                reset = false;
                bot.outtake.state = "transient";
                bot.outtake.update();
            }else if(gamepad2.b) {
                reset = true;
                bot.outtake.setTargets(0, 5, 0, 0);
            }

            angle -= gamepad2.left_stick_x;
            length -= gamepad2.left_stick_y;
            if(gamepad2.dpad_up) {
                tilt += 5;
            }else if(gamepad2.dpad_down) {
                tilt -= 5;
            }

            if(gamepad2.dpad_right) {
                bot.outtake.startAngle += 1;
            }else if(gamepad2.dpad_left) {
                bot.outtake.startAngle -= 1;
            }

            if(!reset && loopCount > 15) {
                bot.outtake.state = "transient";
                bot.outtake.setTargets(angle, tilt, length, 1);
            }

            if(gamepad2.x) {
                bot.outtake.setBoxState(2);
                bot.outtake.state = "transient";
            }else if(gamepad2.y) {
                bot.outtake.setBoxState(0);
                bot.outtake.state = "transient";
            }else if(gamepad2.right_bumper) {
                bot.outtake.setBoxState(1);
                bot.outtake.state = "transient";
            }

            bot.drivebase.update();
            bot.outtake.update();
            bot.intake.update();

            telemetry.addData("angle", angle);
            telemetry.addData("tilt", tilt);
            telemetry.addData("length", length);
            telemetry.addData("reset", reset);

            telemetry.addData("turret", bot.outtake.turretPosition);
            telemetry.addData("slide", bot.outtake.slidePosition);
            telemetry.addData("tilt", bot.outtake.tiltPosition);
            telemetry.addData("startPos", bot.outtake.startAngle);

            bot.update();
            loopCount ++;

            telemetry.update();
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