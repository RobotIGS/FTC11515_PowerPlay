package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareMaps.BaseHardwareMap;
import org.firstinspires.ftc.teamcode.HardwareMaps.FullHardwareMap;
import org.firstinspires.ftc.teamcode.HardwareMaps.GyroHardwareMap;
import org.firstinspires.ftc.teamcode.Tools.FieldNavigation;

@TeleOp
public class FullControl extends BaseTeleOp {
    BaseHardwareMap robot;
    FieldNavigation navi;
    GyroHardwareMap gyro;

    double wy;

    @Override
    public void initialize() {

        robot = new FullHardwareMap(hardwareMap);
        gyro = new GyroHardwareMap(hardwareMap);
        navi = new FieldNavigation(robot, gyro, 0.0,0.0, 0.0, 0.0,0.0);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            robot.motor_lift.setPower(gamepad1.right_stick_y * 0.3);
        } else {
            robot.motor_lift.setPower(0);
        }
        wy = (gamepad1.left_trigger != 0.0) ? gamepad1.left_trigger : -gamepad1.right_trigger;
        navi.drive_setSpeed(gamepad1.left_stick_y,gamepad1.left_stick_x,wy,0.7);
        navi.step();
    }
}
