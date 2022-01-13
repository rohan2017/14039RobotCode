package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.FFRobot;

@TeleOp(name="Drivebase Test", group="Testing")
public class testDrivebase extends LinearOpMode {

    // Declare OpMode Members
    //private FourWheelRobot bot = new FourWheelRobot(this);
    //private MecanumChassisBot bot = new MecanumChassisBot(this);
    private FFRobot bot = new FFRobot(this);

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        telemetry.addData("status","running");
        telemetry.update();

        while(opModeIsActive()) {

            double y1 = -gamepad1.right_stick_y;
            double x1 = gamepad1.right_stick_x;
            double x2 = gamepad1.left_stick_x;
            double y2 = -gamepad1.left_stick_y;

            //bot.drivebase.setPowers((y2+x2), (y1-x1), (y2-x2), (y1+x1));
            bot.drivebase.setPowers(y2, y1);

            if(gamepad1.left_trigger < 0.62) {
                bot.intake.setFlipPosition(gamepad1.left_trigger);
            }
            if(gamepad1.right_trigger < 0.33) {
                bot.intake.setExtendPosition(gamepad1.right_trigger);
            }


            telemetry.addData("leftTrigger", gamepad1.left_trigger);
            telemetry.addData("rightTrigger", gamepad1.right_trigger);

            bot.drivebase.update();
            bot.intake.update();

            telemetry.update();
        }
        bot.drivebase.freeze();
    }

    private void initialize() {
        bot.initialize(hardwareMap);
        bot.movement.state = "transient";
        telemetry.addData("status","initialized");
        telemetry.update();
    }
}