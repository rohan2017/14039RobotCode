package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MathFunctions.PointEx;
import org.firstinspires.ftc.teamcode.Robots.FFRobot;
import org.firstinspires.ftc.teamcode.Robots.FourWheelRobot;
import org.firstinspires.ftc.teamcode.Robots.testBot;
import org.firstinspires.ftc.teamcode.Subsystems.State;

@Autonomous(name="Park Auto", group="OpMode")
public class parkAuto extends LinearOpMode {

    // Declare OpMode Members
    private FFRobot bot = new FFRobot(this);
    private final int allianceTurret = -65;
    private final int allianceSlide = 140;
    private final int allianceTilt = 26;

    public enum AutoMode {
        EXTEND,
        DEPOSIT,
        DUMP,
        HOME,
        Y_RESET,
        HOMING_1,
        HOMING_2,
        HOMING_3,
        MOVING,
        DONE
    }

    private AutoMode autoM = AutoMode.EXTEND;

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        telemetry.addData("status","running");
        telemetry.update();

        while(opModeIsActive()) {
            switch(autoM) {
                case EXTEND:
                    bot.outtake.setTargets(allianceTurret, allianceTilt, allianceSlide, 1);
                    bot.time.delaySeconds(6);
                    autoM = AutoMode.DEPOSIT;
                    break;
                case DEPOSIT:
                    if (bot.time.state == State.CONVERGED) {
                        // Dumping
                        bot.outtake.setBoxState(2); // deposit state
                        bot.time.delaySeconds(1);
                        autoM = AutoMode.DUMP;
                        break;
                    }
                case DUMP: // begin pulling slides in
                    if (bot.time.state == State.CONVERGED) {
                        // Start homing by pulling the slides in
                        bot.outtake.setTargets(bot.outtake.turretPosition, bot.outtake.tiltPosition, 50, 1);
                        bot.time.delaySeconds(0.8); //.2
                        autoM = AutoMode.HOMING_1;
                    }
                    break;
                //HOMING SEQUENCE BEGINS HERE:
                case HOMING_1: // hold slides and rotate turret
                    if (bot.time.state == State.CONVERGED) {
                        // Start homing by pulling the slides in
                        bot.outtake.setTargets(0, 0, 50, 1);
                        bot.time.delaySeconds(0.7); //.2
                        autoM = AutoMode.HOMING_2; //Go to HOMING_2 Stat
                    }
                    break;
                case HOMING_2: //coming into center of robot
                    if (bot.time.state == State.CONVERGED) {
                        // Start homing by pulling the slides in
                        bot.outtake.setTargets(0, 0, 16, 1);
                        bot.time.delaySeconds(0.5);
                        autoM = AutoMode.HOMING_3; //Go HOME
                    }
                    break;

                case HOMING_3:
                    if (bot.time.state == State.CONVERGED) {
                        bot.outtake.setTargets(0, 0, 6, 0); // "reset" manipulator
                        bot.time.delaySeconds(0.6);
                        autoM = AutoMode.HOME;
                    }
                    break;
                case Y_RESET:
                    if (bot.time.state == State.CONVERGED) {
                        bot.outtake.setTargets(0, 0, 10, 0); // "reset" slides
                        bot.time.delaySeconds(0.25);
                        autoM = AutoMode.HOME;
                    }
                    break;
                case HOME:
                    if (bot.time.state == State.CONVERGED) {
                        bot.outtake.setTargets(0, 0, 0, 0);
                        bot.time.delaySeconds(7);
                        autoM = AutoMode.MOVING;
                        bot.movement.setTarget(new PointEx(0, 70, 0));
                    }
                    break;
                case MOVING:
                    if (bot.movement.state == State.CONVERGED || bot.time.state == State.CONVERGED) {
                        autoM = AutoMode.DONE;
                    }
                    break;
                case DONE:
                    bot.drivebase.freeze();
                    bot.outtake.setBoxState(0);
                    bot.outtake.setSlideLength(0);
                    break;
            }
            bot.update();
            telemetry.addData("State", autoM);
        }

    }

    private void initialize() {
        bot.initialize(hardwareMap);
        bot.movement.state = State.TRANSIENT;
        bot.update();
        telemetry.addData("status","initialized");
        telemetry.update();
    }
}