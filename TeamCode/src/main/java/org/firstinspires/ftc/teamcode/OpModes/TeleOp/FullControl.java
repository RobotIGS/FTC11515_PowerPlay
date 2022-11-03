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
        if (gamepad1.a) {
            robot.motor_lift.setPower(gamepad1.right_stick_y);
        } else {
            robot.motor_lift.setPower(0);
        }
    }
}
