package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareMaps.BaseHardwareMap;
import org.firstinspires.ftc.teamcode.HardwareMaps.FullHardwareMap;

import org.firstinspires.ftc.teamcode.HardwareMaps.GyroHardwareMap;
import org.firstinspires.ftc.teamcode.Tools.FieldNavigation;

public class FullControl extends BaseTeleOp {
    BaseHardwareMap robot;
    FieldNavigation navi;
    GyroHardwareMap gyro;

    double wy;

    @Override
    public void initialize() {
        robot = new FullHardwareMap(hardwareMap);
        navi = new FieldNavigation(robot, gyro, 0.0, 0.0, 0.0, 0.7/180, 0.5);
    }

    @Override
    public void loop() {
        wy = (gamepad1.left_trigger != 0.0) ? gamepad1.left_trigger : -gamepad1.right_trigger;
        navi.drive_setSpeed(gamepad1.left_stick_x, gamepad1.left_stick_y, wy, 1.0);
        navi.step();
        telemetry.addData("wy :", wy);
        telemetry.addData("vx :", gamepad1.left_stick_x);
        telemetry.addData("vz :", gamepad1.left_stick_y);
    }
}