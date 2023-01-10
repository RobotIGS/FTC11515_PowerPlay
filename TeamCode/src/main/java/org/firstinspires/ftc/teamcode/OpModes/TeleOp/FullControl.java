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
        if (gamepad1.a) {
            gain += 0.005;
        } else if (gamepad1.b && gain > 1) {
            gain -= 0.005;
        }

         telemetry.addData("Gain", gain);
         robot.colorSensor.setGain(gain);
        NormalizedRGBA colors = robot.colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        telemetry.addLine()
                .addData("Red","%.3f", colors.red)
                .addData("Green","%.3f", colors.green)
                .addData("Blue","%.3f", colors.blue);
        telemetry.addLine()
                .addData("Hue", "%.3f", hsvValues[0])
                .addData("Saturation", "%.3f", hsvValues[1])
                .addData("Value", "%.3f", hsvValues[2]);
        telemetry.addData("Alpha", "%.3f", colors.alpha);

    }
}
