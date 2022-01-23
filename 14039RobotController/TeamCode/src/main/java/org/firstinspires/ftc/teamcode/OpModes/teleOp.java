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

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        telemetry.addData("status","running");
        telemetry.update();

        double length = 0;
        double loopCount = 0;
        boolean reset = false;

        while(opModeIsActive()) {

            // DRIVING
            bot.drivebase.setPowers(-gamepad1.left_stick_y, -gamepad1.right_stick_y);

            // INTAKE
            if(gamepad2.right_bumper) {
                bot.intake.setPower(-0.8);
            }else if(gamepad2.right_trigger > 0.1) {
                bot.intake.setPower(0.8);
            }else {
                bot.intake.setPower(0);
            }
            // flipping
            if(gamepad2.left_stick_y < -0.1) {
                bot.intake.flipDown();
            }else if(gamepad2.left_stick_y > 0.1) {
                bot.intake.flipUp();
            }else {
                bot.intake.flipHold();
            }
            // extend
            bot.intake.setExtendPosition(gamepad2.left_trigger);

            // OUTTAKE
            // Extension manual control
            length -= gamepad2.left_stick_y;

            // Turret manual control
            if(gamepad2.right_stick_x > 0.05) {
                bot.outtake.setTurretPower(-(gamepad2.right_stick_x - 0.05)*0.3);
            }else if(gamepad2.right_stick_x < -0.05) {
                bot.outtake.setTurretPower(-(gamepad2.right_stick_x + 0.05)*0.3);
            }else if(!reset){
                bot.outtake.setTurretPower(0);
            }

            // Tilt control
            if(gamepad2.dpad_up) {
                bot.outtake.setTiltPower(0.2);
            }else if(gamepad2.dpad_down) {
                bot.outtake.setTiltPower(-0.2);
            }else if(!reset){
                bot.outtake.setTiltPower(0);
            }

            if(gamepad2.dpad_right) {
                bot.outtake.startAngle += 1;
            }else if(gamepad2.dpad_left) {
                bot.outtake.startAngle -= 1;
            }

            if(loopCount > 3) {
                bot.outtake.setSlideLength(length);
            }

            if(gamepad2.a) {
                loopCount = 0;
                length = 50;
                reset = true;
                bot.outtake.setTargets(10, 10, length, 1);
            }else if(gamepad2.b) {
                length = 0;
                reset = true;
                bot.outtake.setTargets(0, 0, length, 1);
            }

            if(bot.outtake.state == State.CONVERGED) {
                reset = false;
            }

            if(gamepad2.x) {
                bot.outtake.setBoxState(2);
            }else if(gamepad2.y) {
                bot.outtake.setBoxState(0);
            }else if(gamepad2.right_bumper) {
                bot.outtake.setBoxState(1);
            }

            bot.update();

            /*
            telemetry.addData("turret", bot.outtake.turretPosition);
            telemetry.addData("turret mode", bot.outtake.turretMode);

            telemetry.addData("tilt", bot.outtake.tiltPosition);
            telemetry.addData("tilt mode", bot.outtake.tiltMode);

            telemetry.addData("slide", bot.outtake.slidePosition);

            telemetry.addData("servo", bot.outtake.getServoState());

            telemetry.addData("pid", bot.hardware.getMotor("extension").getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            */
            telemetry.addData("intaked cargo", bot.intake.hasBlock);
            telemetry.addData("intensity", bot.intake.filteredIntensity);

            telemetry.update();

            loopCount ++;
        }
       bot.drivebase.freeze();
    }

    private void initialize() {
        bot.initialize(hardwareMap);
        // bot.drivebase.setRunMode("withoutEncoder");
        telemetry.addData("status","initialized");
        telemetry.update();
    }
}