package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.List;

public class testHardware extends RobotHardware {

    //Drive Motors
    public static DcMotorEx rightFront, leftFront, leftBack, rightBack;

    //Intake
    public static DcMotorEx intake;
    public static Servo rightFlipper, leftFlipper, leftExtend,rightExtend;
    public static ColorRangeSensor intakeRange;

    //Outtake
    public static DcMotorEx turret, tilt, extension;
    public static Servo basket;
    // public static AnalogInput potentiometer;

    //IMU
    public static BNO055IMU imu;

    //Timer
    public static ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    //Camera
    public static int cameraMonitorViewId;

    public static List<LynxModule> allHubs;

    @Override
    public void hardwareMap(HardwareMap hardwareMap) {

        //IMU
        imu =  hardwareMap.get(BNO055IMU.class, "imu");

        //Intake
        rightFlipper = hardwareMap.get(Servo.class, "rightFlipper");
        leftFlipper = hardwareMap.get(Servo.class, "leftFlipper");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        leftExtend = hardwareMap.get(Servo.class, "leftExtend");
        rightExtend = hardwareMap.get(Servo.class, "rightExtend");
        intakeRange = hardwareMap.get(ColorRangeSensor.class, "intake_range");

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");


        //Camera
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        allHubs = hardwareMap.getAll(LynxModule.class);

    }

    @Override
    public void initialize() {

        BNO055IMU.Parameters Params = new BNO055IMU.Parameters();
        Params.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        Params.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        Params.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opMode
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
            case "driveBackRight":
                return rightBack;
            case "driveBackLeft":
                return leftBack;
            case "intake":
                return intake;
            case "turret":
                return turret;
            case "tilt":
                return tilt;
            case "extension":
                return extension;
            default:
                return null;
        }
    }

    public Servo getServo(String ID){
        switch (ID){
            case "leftFlipper":
                return leftFlipper;
            case "rightFlipper":
                return rightFlipper;
            case "leftExtend":
                return leftExtend;
            case "rightExtend":
                return rightExtend;
            case "basketFlipper":
                return basket;
            default:
                return null;
        }
    }


    public ColorRangeSensor getRangeSensor(String ID) {
        switch (ID) {
            case "intake_range":
                return intakeRange;
            default:
                return null;
        }
    }

    public int getCameraID(String ID) {
        return cameraMonitorViewId;
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