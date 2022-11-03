package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareMaps.BaseHardwareMap;
import org.firstinspires.ftc.teamcode.HardwareMaps.FullHardwareMap;

@TeleOp
public class ServoTest extends BaseTeleOp {
    BaseHardwareMap robot;
    @Override
    public void initialize() {
        robot = new FullHardwareMap(hardwareMap);
    }
    @Override
    public void loop() {
        if (gamepad1.x) {
            robot.servo1.setPosition(0.0);
            robot.servo2.setPosition(0.4);
        }
        if (gamepad1.y) {
            robot.servo1.setPosition(0.4);
            robot.servo2.setPosition(0.0);
        }
    }
}

