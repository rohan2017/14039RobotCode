package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class RobotHardware {

    public void hardwareMap(HardwareMap hardwareMap) {}
    public void initialize() {}

    // Hardware retriever methods
    public BNO055IMU getImu(String ID) {return null;}
    public double getImuHeading(String ID) {return 0.0;}
    public double getImuHeading() {return 0.0;}
    public double[] getImuAcc() {return new double[2];}

    public DcMotorEx getMotor(String ID) {return null;}
    public Servo getServo(String ID) {return null;}
    public CRServo getCRServo(String ID) {return null;}
    public AnalogInput getAnalogInput(String ID) {return null;}
    public DigitalChannel getDigitalInput(String ID) {return null;}
    public ColorRangeSensor getRangeSensor(String ID) {return null;}

    public void resetTimer() {}
    public double getTime() {return 0.0;}

    public int getCameraID(String ID) {return 0;}

}