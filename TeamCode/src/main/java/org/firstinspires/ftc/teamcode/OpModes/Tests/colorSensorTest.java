package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@TeleOp(name="Color Sensor Distance Test", group="Testing")
public class colorSensorTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Define a variable for Color Sensor V3
        ColorRangeSensor color;

            // Get the color sensor from hardwareMap
            color = hardwareMap.get(ColorRangeSensor.class, "intakeSensor");

            // Wait for the Play button to be pressed
            waitForStart();

            // While the Op Mode is running, update the telemetry values.
            while (opModeIsActive()) {
                telemetry.addData("Raw Light Detected", color.getRawLightDetected());
                telemetry.addData("getDistance method", color.getDistance(DistanceUnit.MM));
                //telemetry.addData("Blue", color.getDistance(CM));
                telemetry.update();
            }
    }

    private void initialize() {

        telemetry.addData("status", "initialized");
        telemetry.update();
    }
}


