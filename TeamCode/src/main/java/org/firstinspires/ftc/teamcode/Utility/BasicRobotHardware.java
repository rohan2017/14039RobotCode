package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BasicRobotHardware extends RobotHardware {

    //Drive Motors
    public static DcMotor rightFront, leftFront, leftBack, rightBack; //Drive Motor Objects
    //IMU
    public static BNO055IMU imu;

    public void hardwareMap(HardwareMap hardwareMap) {

        //Drive-train
        rightFront = hardwareMap.dcMotor.get("driveFrontRight");
        leftFront = hardwareMap.dcMotor.get("driveFrontLeft");
        leftBack = hardwareMap.dcMotor.get("driveBackLeft");
        rightBack = hardwareMap.dcMotor.get("driveBackRight");

        //IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters Params = new BNO055IMU.Parameters();
        Params.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        Params.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        Params.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opMode
        Params.loggingEnabled      = true;
        Params.loggingTag          = "IMU";
        Params.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(Params);

    }

    public BNO055IMU getImu(String ID) {
        return imu;
    }

    public DcMotor getMotor(String ID) {
        switch (ID) {
            case "driveFrontRight":
                return rightFront;
            case "driveFrontLeft":
                return leftFront;
            case "driveBackRight":
                return rightBack;
            case "driveBackLeft":
                return leftBack;
            default:
                return null;
        }

    }

}
