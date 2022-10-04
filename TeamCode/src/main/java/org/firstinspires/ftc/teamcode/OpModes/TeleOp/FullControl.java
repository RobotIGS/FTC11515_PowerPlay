package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareMaps.BaseHardwareMap;
import org.firstinspires.ftc.teamcode.HardwareMaps.FullHardwareMap;

public class FullControl extends BaseTeleOp {
    BaseHardwareMap robot;

    @Override
    public void initialize() {
        robot = new FullHardwareMap(hardwareMap);
    }

    @Override
    public void loop() {
    }
}
