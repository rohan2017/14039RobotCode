package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.CustomCV.Detector.Location.LEFT;
import static org.firstinspires.ftc.teamcode.CustomCV.Detector.Location.MID;
import static org.firstinspires.ftc.teamcode.CustomCV.Detector.Location.RIGHT;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.FourWheelRobot;
import org.firstinspires.ftc.teamcode.Robots.testBot;
import org.openftc.easyopencv.OpenCvCamera;

import org.firstinspires.ftc.teamcode.CustomCV.Detector;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


@Autonomous(name="Intake Auto", group="Testing")

public class intakeAuto extends LinearOpMode
{
    // Declare OpMode Members
    private testBot bot = new testBot(this);

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        telemetry.addData("status","running");
        telemetry.update();
    }


    private void initialize() {
        bot.initialize(hardwareMap);
        bot.update();
        telemetry.addData("status","initialized");
        telemetry.update();
    }
}

