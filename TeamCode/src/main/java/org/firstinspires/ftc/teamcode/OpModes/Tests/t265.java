package org.firstinspires.ftc.teamcode.OpModes.Tests;


import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.spartronics4915.lib.T265Camera;

@TeleOp(name="Test T265", group="Tests")
@Disabled
public class t265 extends OpMode
{
    private static T265Camera slamra = null;
    Pose2d startingPose = new Pose2d(1, 1, new Rotation2d());


    @Override
    public void init() {
        if (slamra == null) {
            slamra = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);
            slamra.setPose(startingPose);
        }
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        slamra.start();
    }

    @Override
    public void loop() {

        T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();

        if (up == null) {
            return;
        }

        telemetry.addData("data", up);
        telemetry.update();

    }

    @Override
    public void stop() {
        slamra.stop();
    }

}