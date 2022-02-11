package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controllers.PIDF;
import org.firstinspires.ftc.teamcode.Robots.FFRobot;
import org.firstinspires.ftc.teamcode.Subsystems.State;

@TeleOp(name="outtake test", group="TeleOp")
public class outtakeTest extends LinearOpMode {

    // Declare OpMode Members
    private FFRobot bot = new FFRobot(this);
    private double kP, kI, kD, iLim;
    private PIDF turretControl = new PIDF(0,0,0, 0.01, 0,0.4,0);
    private double targetTurret;

    private boolean lastRight;
    private boolean lastLeft;
    private boolean lastUp;
    private boolean lastDown;

    private int mode;
    private String modeStr;

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        telemetry.addData("status", "running");
        telemetry.update();

        // OUTTAKE
        while (opModeIsActive()) {
            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;
            boolean left = gamepad1.dpad_left;
            boolean right = gamepad1.dpad_right;

            bot.outtake.state = State.TRANSIENT;

            //telemetry.addData("outtake state", bot.outtake.state);
            //telemetry.addData("ready receive", bot.outtake.readyReceive);

            telemetry.addData("turret ticks", bot.outtake.turretPosition);
            telemetry.addData("turret target", targetTurret);

            telemetry.addData("P", kP);
            telemetry.addData("I", kI);
            telemetry.addData("D", kD);
            telemetry.addData("lim", iLim);
            telemetry.addData("Adjusting", modeStr);

            /*
            telemetry.addData("tilt angle", bot.outtake.tiltPosition);
            telemetry.addData("tilt target", bot.outtake.targetTiltPosition);
            telemetry.addData("tilt error", bot.outtake.tiltError);

            telemetry.addData("slide length", bot.outtake.getSlideLength());
            telemetry.addData("slide ticks", bot.outtake.slidePosition);
            telemetry.addData("slide error", bot.outtake.slideError);
            */

            if(gamepad1.a) {
                targetTurret = 0;
            }else if(gamepad1.b) {
                targetTurret = 100;
            }else if(gamepad1.x) {
                targetTurret = -100;
            }

            // P - 0
            // I - 1
            // D - 2
            // iLIM - 3
            if(!right && lastRight) {
                mode ++;
            }else if(!left && lastLeft) {
                mode --;
            }
            if(mode < 0) mode = 3;
            if(mode > 3) mode = 0;

            switch (mode) {
                case 0:
                    modeStr = "P";
                    break;
                case 1:
                    modeStr = "I";
                    break;
                case 2:
                    modeStr = "D";
                    break;
                case 3:
                    modeStr = "I limit";
                    break;
            }

            if(!up && lastUp) {
                switch (mode) {
                    case 0:
                        kP += 0.0001;
                        break;
                    case 1:
                        kI += 0.0001;
                        break;
                    case 2:
                        kD += 0.0001;
                        break;
                    case 3:
                        iLim += 0.001;
                        break;
                }
            }else if(!down && lastDown) {
                switch (mode) {
                    case 0:
                        kP -= 0.0001;
                        break;
                    case 1:
                        kI -= 0.0001;
                        break;
                    case 2:
                        kD -= 0.0001;
                        break;
                    case 3:
                        iLim -= 0.001;
                        break;
                }
            }

            turretControl.updateGains(kP, kI, kD, 0.002, iLim);
            turretControl.update(targetTurret, bot.outtake.turretPosition);
            bot.outtake.setTurretPower(turretControl.correction);

            telemetry.update();
            bot.update();

            lastUp  = up;
            lastDown = down;
            lastLeft = left;
            lastRight = right;
        }

    }

    private void initialize() {
        bot.initialize(hardwareMap);
        bot.update();
        targetTurret = 0;

        kP = 0.004;
        kI = 0;
        kD = 0.0005;
        iLim = 0;

        mode = 0;
        modeStr = "P";

        telemetry.addData("status","initialized");
        telemetry.update();
    }
}

