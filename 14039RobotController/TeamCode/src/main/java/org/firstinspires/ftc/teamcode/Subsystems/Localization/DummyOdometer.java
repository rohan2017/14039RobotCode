package org.firstinspires.ftc.teamcode.Subsystems.Localization;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class DummyOdometer extends Odometer {

    private LinearOpMode opMode;
    private RobotHardware hardware;

    private double lastHeadingImu;
    private double headingImu;

    public DummyOdometer(LinearOpMode opMode, RobotHardware robothardware) {
        this.opMode = opMode;
        this.hardware = robothardware;
    }

    public void initialize() {
        headingImu = 0;
        lastHeadingImu = headingImu;
    }

    public void update() {
        if(opMode.opModeIsActive()) {

            headingImu = hardware.getImuHeading("hub1");

            // Math Variables
            double headingChange = headingImu - lastHeadingImu;
            //headingChange *= 1.013; // Weird IMU calibration issue

            if (headingChange < -Math.PI){ // For example 355 to 2 degrees
                headingChange = 2*Math.PI + headingChange;
            }else if (headingChange > Math.PI) { // For example 2 to 355 degrees
                headingChange = -2*Math.PI + headingChange;
            }

            headingRadians += headingChange;
            heading = Math.toDegrees(headingRadians);

            lastHeadingImu = headingImu;
        }
    }
}
