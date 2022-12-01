package org.firstinspires.ftc.teamcode.HardwareMaps;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

public abstract class BaseHardwareMap {
    public HardwareMap hwMap;

    public DcMotor motor_front_right;
    public DcMotor motor_front_left;
    public DcMotor motor_rear_right;
    public DcMotor motor_rear_left;
    public NormalizedColorSensor colorSensor;
    public BaseHardwareMap(HardwareMap hwMap) {
        this.hwMap = hwMap;
        init(hwMap);
    }

    public abstract void init(HardwareMap hwMap);
}
