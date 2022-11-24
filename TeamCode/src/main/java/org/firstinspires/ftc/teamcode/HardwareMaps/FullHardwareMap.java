package org.firstinspires.ftc.teamcode.HardwareMaps;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FullHardwareMap extends BaseHardwareMap {
    public FullHardwareMap(HardwareMap hwMap) {
        super(hwMap);
    }

    @Override
    public void init(HardwareMap hwMap) {
        motor_front_left = hwMap.get(DcMotor.class, "hub1_motorport0");
        motor_front_right = hwMap.get(DcMotor.class, "hub1_motorport1");
        motor_rear_left = hwMap.get(DcMotor.class, "hub1_motorport2");
        motor_rear_right = hwMap.get(DcMotor.class, "hub1_motorport3");
    }
}
