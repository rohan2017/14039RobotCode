package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.FFRobot;
import org.firstinspires.ftc.teamcode.Robots.MecanumChassisBot;

@Autonomous(name="teleOp", group="TeleOp")
public class teleOp extends LinearOpMode {

    // Declare OpMode Members
    private MecanumChassisBot bot = new MecanumChassisBot(this);

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        telemetry.addData("status","running");
        telemetry.update();

        while(opModeIsActive()) {

            // DRIVING
            double speed = gamepad1.left_bumper ? 0.5 : 1;
            double y1 = -gamepad1.right_stick_y * speed;
            double x1 = -gamepad1.right_stick_x * speed;
            double x2 = -gamepad1.left_stick_x * speed;
            double y2 = -gamepad1.left_stick_y * speed;
            bot.drivebase.setPowers((y2-x2), (y1+x1), (y2+x2), (y1-x1));

            if(gamepad1.a) {
                bot.drivebase.setPowers(0.5, 0.5, 0.5, 0.5);
            }

            // INTAKE
            if(gamepad2.left_trigger > 0.5){
                bot.intake.setPower(0.8);
            }else if(gamepad2.right_trigger > 0.5) {
                bot.intake.setPower(-0.8);
            } else {
                bot.intake.setPower(0);
            }

            // LIFT
            if(gamepad2.dpad_up) {
                bot.outtake.setLiftPower(1);
            }else if(gamepad2.dpad_down) {
                bot.outtake.setLiftPower(-0.5);
            }else {
                bot.outtake.setLiftPower(0);
            }

            // ARM
            if(gamepad2.dpad_right) {
                bot.outtake.setArmPower(0.1);
            }else if (gamepad2.dpad_left) {
                bot.outtake.setArmPower(-0.1);
            }else {
                bot.outtake.setArmPower(0);
            }

            // DOOR
            if(gamepad2.a){
                bot.outtake.doorState("open");
            }else {
                bot.outtake.doorState("closed");
            }


            // PUSHER
            if (gamepad2.b) {
                //
            }

            // ROTATOR
            if (gamepad2.left_bumper) {
                bot.outtake.rotatorPosition -= 0.05;
            }else if(gamepad2.right_bumper) {
                bot.outtake.rotatorPosition += 0.05;
            }else if(gamepad2.y) {
                bot.outtake.rotatorPosition = 0;
            }else if(gamepad2.x) {
                bot.outtake.rotatorPosition = 0.8;
            }
            bot.update();

            telemetry.update();
        }
        bot.drivebase.freeze();
    }

    private void initialize() {
        bot.initialize(hardwareMap);
        telemetry.addData("status","initialized");
        telemetry.update();
    }
}