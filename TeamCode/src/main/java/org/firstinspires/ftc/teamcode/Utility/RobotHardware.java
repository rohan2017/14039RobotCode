package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class RobotHardware {

    public void hardwareMap(HardwareMap hardwareMap) {}

    public BNO055IMU getIMU(String ID){return null;}
    public DcMotor getMotor(String ID){return null;}
    public Servo getServo(String ID){return null;}

}