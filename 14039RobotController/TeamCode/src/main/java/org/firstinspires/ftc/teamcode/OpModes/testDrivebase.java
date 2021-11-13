package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.MecanumChassisBot;

@Autonomous(name="Drivebase Test", group="Tests")
public class testDrivebase extends LinearOpMode {

    // Declare OpMode Members
    private MecanumChassisBot bot = new MecanumChassisBot(this);

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        telemetry.addData("status","running");
        telemetry.update();

        while(opModeIsActive()) {
            if(gamepad1.left_bumper) {
                double y1 = -gamepad1.right_stick_y;
                double x1 = -gamepad1.right_stick_x;
                double x2 = -gamepad1.left_stick_x;
                double y2 = -gamepad1.left_stick_y;

                bot.drivebase.setPowers((y2-x2), (y1+x1), (y2+x2), (y1-x1));
            }else {
                bot.drivebase.setRelativeVelocity(gamepad1.left_stick_x*5, -gamepad1.left_stick_y*5, -gamepad1.right_stick_x*5);
            }
            bot.drivebase.update();
            telemetry.addData("JoyX",gamepad1.right_stick_x);
            telemetry.addData("JoyY",-gamepad1.right_stick_y);
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