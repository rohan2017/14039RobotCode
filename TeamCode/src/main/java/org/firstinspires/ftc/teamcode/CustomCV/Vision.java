package org.firstinspires.ftc.teamcode.CustomCV;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class Vision {

    private LinearOpMode opMode;
    private RobotHardware hardware;
    public Detector detector;

    private OpenCvCamera camera;
    public Vision(LinearOpMode opMode, RobotHardware hardware) {
        this.opMode = opMode;
        this.hardware = hardware;
        this.detector = new Detector();
    }

    public void initialize() {
        int cameraMonitorViewId = hardware.getCameraID("");
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        Detector detector = new Detector();
        camera.setPipeline(detector);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }

            @Override
            public void onError(int errorCode) {
                opMode.telemetry.addData("error","Failed to open camera.");
                opMode.telemetry.update();
            }
        });
    }

    public void stop() {
        camera.stopStreaming();
    }

}
