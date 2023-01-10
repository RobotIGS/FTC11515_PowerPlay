package org.firstinspires.ftc.teamcode.HardwareMaps;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.ColorSensor; // test, ob das ansatt normalized
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public abstract class BaseHardwareMap {
    public HardwareMap hwMap;

    public DcMotor motor_front_right;
    public DcMotor motor_front_left;
    public DcMotor motor_rear_right;
    public DcMotor motor_rear_left;
    public DcMotor motor_lift;

    public Servo servo1;
    public Servo servo2;
    public Servo servo3;



    public BaseHardwareMap(HardwareMap hwMap) {
        this.hwMap = hwMap;
        init(hwMap);
    }

    public abstract void init(HardwareMap hwMap);
}
