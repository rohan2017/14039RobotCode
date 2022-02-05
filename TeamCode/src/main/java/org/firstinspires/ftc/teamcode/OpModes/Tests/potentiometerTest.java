package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@Disabled
@TeleOp(name="Potentiometer Test", group="Testing")
public class potentiometerTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        telemetry.addData("status", "running");
        telemetry.update();

        AnalogInput potentiometer = hardwareMap.analogInput.get("slidePivotPot");

        while (opModeIsActive()) {
            double voltreading = (float) potentiometer.getVoltage();
            double percentTurned = voltreading/3.3 * 100;

            telemetry.addData("potentiometer voltage ", voltreading);
            telemetry.addData("percent rotated", percentTurned);
            telemetry.update();

        }
    }

    private void initialize() {

        telemetry.addData("status", "initialized");
        telemetry.update();
    }
}


