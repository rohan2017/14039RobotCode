package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.usb.serial.RobotUsbDeviceTty;

import org.firstinspires.ftc.teamcode.Robots.FFRobot;
import org.firstinspires.ftc.teamcode.Robots.FourWheelRobot;
import org.firstinspires.ftc.teamcode.Robots.MecanumChassisBot;

@TeleOp(name="outtake test", group="TeleOp")
public class outtakeTest extends LinearOpMode {

    // Declare OpMode Members
    private FourWheelRobot bot = new FourWheelRobot(this);

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        telemetry.addData("status", "running");
        telemetry.update();

        // OUTTAKE
        double angle = 70;
        double length = 0;
        double tilt = 0;

        bot.outtake.setTargets(angle, tilt, length, 1);
        while (opModeIsActive()) {
            telemetry.addData("turret ticks ", bot.hardware.getMotor("turret").getCurrentPosition());
            telemetry.addData("state", bot.outtake.state);
            telemetry.addData("turret mode", bot.outtake.turretMode);
            telemetry.addData("turret power", bot.outtake.turretPower);
            telemetry.addData("turret error", bot.outtake.turretError);
            telemetry.addData("turret targ in ticks", bot.outtake.targetTurretPosition);
            telemetry.addData("turret targ angle", bot.outtake.targetTurretPosition/ bot.outtake.ticksPerDegTurret);
            telemetry.addData("ticks per deg turret", bot.outtake.ticksPerDegTurret);
            telemetry.update();
            bot.outtake.update();
        }

    }



    private void initialize() {
        bot.initialize(hardwareMap);
        bot.drivebase.setRunMode("withoutEncoder");
        telemetry.addData("status","initialized");
        telemetry.update();
    }
}

