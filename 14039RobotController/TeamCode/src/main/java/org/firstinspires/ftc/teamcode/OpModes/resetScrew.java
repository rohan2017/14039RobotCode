package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.FourWheelRobot;

@TeleOp(name="reset Tilt", group="TeleOp")
public class resetScrew extends LinearOpMode {

    // Declare OpMode Members
    private FourWheelRobot bot = new FourWheelRobot(this);


    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        telemetry.addData("status","running");
        telemetry.update();

        int angle = 0;

        while(opModeIsActive()) {

            if(gamepad2.dpad_up) {
                bot.hardware.getMotor("tilt").setPower(0.3);
            }else if(gamepad2.dpad_down) {
                bot.hardware.getMotor("tilt").setPower(-0.3);
            }

            telemetry.addData("tiltPos", bot.hardware.getMotor("tilt").getCurrentPosition());
            telemetry.addData("targetPos", angle);

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