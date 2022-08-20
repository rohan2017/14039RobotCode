package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.FFRobotHardware;
import org.firstinspires.ftc.teamcode.OpModes.AutoBlue;
import org.firstinspires.ftc.teamcode.OpModes.teleOpBLUE;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Localization.DummyOdometer;
import org.firstinspires.ftc.teamcode.Subsystems.Localization.Odometer6WD;
import org.firstinspires.ftc.teamcode.Subsystems.Movement.Drivebases.DummyTankDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Movement.MovementDummyTankDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Movement.MovementTankDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Subsystems.State;
import org.firstinspires.ftc.teamcode.Subsystems.Timer;


public class FFRobot extends Robot {

    public DummyTankDrive drivebase;
    public MovementTankDrive movement;
    public Odometer6WD odometer;
    public Intake intake;
    public Outtake outtake;
    public Timer time;
    private LinearOpMode opMode;

    public enum bMode {
        PRIMETRANSFER,
        TRANSFER,
        PRIMEHOLD,
        SETTLEHOLD,
        HOLDING,
        EXTEND,
        DEPOSIT,
        HOMESLIDE,
        HOMETURRET,
        HOMECENTER,
        HOME,
        READY,
        EJECT
    }

    public bMode botMode;

    public FFRobot(LinearOpMode opMode) {
        super(opMode);
        this.opMode = opMode;
        this.hardware = new FFRobotHardware();
        this.drivebase = new DummyTankDrive(opMode, hardware);
        this.odometer = new Odometer6WD(opMode, hardware);
        this.movement = new MovementTankDrive(opMode, drivebase, odometer);
        this.intake = new Intake(opMode, hardware);
        this.outtake = new Outtake(opMode, hardware);
        this.time = new Timer(opMode,hardware);
    }

    public void initialize(HardwareMap hardwareMap) {
        hardware.hardwareMap(hardwareMap);
        hardware.initialize();
        odometer.initialize();
        drivebase.initialize();
        drivebase.resetDriveEncoders();
        drivebase.reverseMotors(1, -1, -1, 1);
        drivebase.setPowerBehavior("brake");
        drivebase.setRunMode("frontWithEncoder");
        movement.initialize();
        outtake.initialize();
        intake.initialize();
        time.initialize();
    }

    public void update() {
        time.update();
        intake.update();
        outtake.update();
        odometer.update();
        movement.update();
        drivebase.update();

    }

    public void teleUpdate() {
        time.update();
        intake.update();
        outtake.update();
        odometer.update();
        drivebase.update();
    }

    public void homeSlides() {
        time.delaySeconds(0.8);
        while(!outtake.homeSlides() && opMode.opModeIsActive()) {
            update();
            if(time.state == State.CONVERGED) {
                time.delaySeconds(0.4);
                while(time.state != State.CONVERGED && opMode.opModeIsActive()) {
                    outtake.setSlidePower(1);
                    update();
                }
                time.delaySeconds(0.8);
            }
        }
    }

    private double dropTurret;
    private double dropTilt;
    private double dropSlide;
    private double extendDelay;
    private int dropPos;

    public void updateDropPosition(double dropTurret, double dropTilt, double dropSlide, int dropPos, double extendDelay) {
        this.dropTurret = dropTurret;
        this.dropTilt = dropTilt;
        this.dropSlide = dropSlide;
        this.dropPos = dropPos;
        this.extendDelay = extendDelay;
    }

    public void stateMachine(boolean extendCondition, boolean dropCondition, boolean homeCondition, boolean transferCondition) {
        intake.setExtendPosition(0.2); // Deployed Position
        switch(botMode) {
            case PRIMETRANSFER:
                // Extrude out intake and fLip up
                outtake.setTargets(0, 0, 0, 0);
                outtake.setSlidePower(-0.3);
                intake.flipUp();

                if (time.state == State.CONVERGED) {
                    time.delaySeconds(0.8); // delay is duration of the next state
                    botMode = bMode.TRANSFER;
                }
                break;
            case TRANSFER:
                // Run motor
                outtake.setTargets(0, 0, 0, 0);
                outtake.setSlidePower(-0.4);
                intake.flipUp();
                intake.setPower(-1);

                if (time.state == State.CONVERGED) {
                    time.delaySeconds(0.3); // delay is duration of the next state
                    intake.setPower(0);
                    botMode = bMode.PRIMEHOLD;
                }
                break;
            case PRIMEHOLD:
                // Retract slide slightly now with block
                outtake.setTargets(0, 0, 1, 0);
                intake.setFlipPosition(0.7);
                if (time.state == State.CONVERGED) {
                    time.delaySeconds(0.4); // delay is duration of the next state
                    botMode = bMode.SETTLEHOLD;
                }
                break;
            case SETTLEHOLD:
                // Let block settle and flip to intermediate hold
                outtake.setTargets(dropTurret, 0, 4, 3);
                intake.setFlipPosition(0.7);

                if (time.state == State.CONVERGED) {
                    time.delaySeconds(0.3); // delay is duration of the next state
                    botMode = bMode.HOLDING;
                }
                break;
            case HOLDING:
                // Flip to hold pos and bucket past walls
                outtake.setTargets(dropTurret, 2, 20, 1);
                intake.setFlipPosition(0.7);

                if (time.state == State.CONVERGED && extendCondition) {
                    time.delaySeconds(extendDelay); // delay is duration of the next state
                    botMode = bMode.EXTEND;
                }
                break;
            case EXTEND:
                // Out to drop-off position
                outtake.setTargets(dropTurret, dropTilt, dropSlide, 1);
                intake.setFlipPosition(0.7);
                outtake.update();
                if ((time.state == State.CONVERGED || outtake.state == State.CONVERGED) && outtake.getSlideLength() > 10 && dropCondition) {
                    time.delaySeconds(0.15); // delay is duration of the next state
                    botMode = bMode.DEPOSIT;
                }
                break;
            case DEPOSIT:
                // Drop block
                outtake.setBoxState(dropPos);
                intake.setFlipPosition(0.7);

                if (time.state == State.CONVERGED && homeCondition) {
                    time.delaySeconds(extendDelay*0.5); // delay is duration of the next state
                    botMode = bMode.HOMESLIDE;
                }
                break;
            case HOMESLIDE:
                // Retract slides and nothing else
                outtake.setTargets(outtake.getTurretAngle(), outtake.tiltPosition, Math.min(43, outtake.getSlideLength()), 1);
                // Flip hold intake
                intake.flipHold();

                if (time.state == State.CONVERGED || outtake.state == State.CONVERGED) {
                    time.delaySeconds(extendDelay*0.4); // delay is duration of the next state
                    botMode = bMode.HOMETURRET;
                }
                break;
            case HOMETURRET:
                // Turret and tilt
                outtake.setTargets(0, 0, Math.min(43, outtake.getSlideLength()), 1);

                if (time.state == State.CONVERGED || outtake.state == State.CONVERGED) {
                    time.delaySeconds(0.6); // delay is duration of the next state
                    botMode = bMode.HOMECENTER;
                }
                break;
            case HOMECENTER:
                // Home slides
                outtake.setTargets(0, 0, 0, 1);

                if (time.state == State.CONVERGED || outtake.state == State.CONVERGED) {
                    time.delaySeconds(0.2); // delay is duration of the next state
                    botMode = bMode.HOME;
                }
                break;
            case HOME:
                // Home bucket
                outtake.setTargets(0, 0, 0, 0);
                if (time.state == State.CONVERGED || outtake.readyReceive) {
                    time.delaySeconds(5);
                    botMode = bMode.READY;
                }
                break;

            case READY:
                outtake.setSlidePower(0);
                if (outtake.readyReceive && transferCondition) {
                    time.delaySeconds(0.7); // delay is duration of the next state
                    botMode = bMode.PRIMETRANSFER;
                }
                break;
            case EJECT:
                intake.flipDown();
                intake.setPower(-1);
                if(time.state == State.CONVERGED) {
                    botMode = bMode.READY;
                }
                break;
        }
    }

}
