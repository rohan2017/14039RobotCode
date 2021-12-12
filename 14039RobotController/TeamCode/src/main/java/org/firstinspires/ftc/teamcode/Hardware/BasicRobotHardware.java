package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.util.List;

public class BasicRobotHardware extends RobotHardware {

    //Drive Motors
    public static DcMotorEx rightFront, leftFront, leftBack, rightBack;
    //Odo Encoders
    public static DcMotorEx parallelEncoder, perpendicularEncoder;
    //IMU
    public static BNO055IMU imu;
    //Timer
    public static ElapsedTime elapsedTime = new ElapsedTime();

    public static List<LynxModule> allHubs;

    @Override
    public void hardwareMap(HardwareMap hardwareMap) {

        //Drive-train
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");



        //IMU
        imu =  hardwareMap.get(BNO055IMU.class, "imu");

        allHubs = hardwareMap.getAll(LynxModule.class);

    }

    @Override
    public void initialize() {

        BNO055IMU.Parameters Params = new BNO055IMU.Parameters();
        Params.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        Params.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        Params.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opMode
        Params.loggingEnabled      = true;
        Params.loggingTag          = "IMU";
        Params.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(Params);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        elapsedTime.startTime();

    }

    @Override
    public BNO055IMU getImu(String ID) {
        return imu;
    }

    @Override
    public double getImuHeading(String ID) {
        if(ID.equals("hub1")) {
            //May need to change axis unit to work with vertical hubs
            Orientation angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            double heading = (angles.firstAngle + 360) % 360;
            return Math.toRadians(heading);
        }else {
            return 0.0;
        }
    }

    public double[] getImuAcc() {
        return new double[]{imu.getLinearAcceleration().xAccel, imu.getLinearAcceleration().yAccel};
    }

    @Override
    public DcMotorEx getMotor(String ID) {
        switch (ID) {
            case "driveFrontRight":
                return rightFront;
            case "driveFrontLeft":
                return leftFront;

            case "driveBackLeft":
            case "verticalEncoder":
                return leftBack;
            case "driveBackRight":
            case "horizontalEncoder":
                return rightBack;
            default:
                return null;
        }

    }

    @Override
    public void resetTimer() {
        elapsedTime.reset();
    }
    @Override
    public double getTime() { // Returns time in milliseconds
        return elapsedTime.milliseconds();
    }

}