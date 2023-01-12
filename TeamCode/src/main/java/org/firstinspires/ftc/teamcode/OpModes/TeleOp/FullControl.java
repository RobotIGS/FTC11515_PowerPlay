package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.HardwareMaps.BaseHardwareMap;
import org.firstinspires.ftc.teamcode.HardwareMaps.FullHardwareMap;
@TeleOp
public class FullControl extends BaseTeleOp {
    BaseHardwareMap robot;

    float gain = 2;
    final float[] hsvValues = new float[3];
    @Override
    public void initialize() {
        robot = new FullHardwareMap(hardwareMap);
    }

    @Override
    public void loop() {
    }
}
