package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareMaps.BaseHardwareMap;
import org.firstinspires.ftc.teamcode.HardwareMaps.FullHardwareMap;
import org.firstinspires.ftc.teamcode.HardwareMaps.GyroHardwareMap;
import org.firstinspires.ftc.teamcode.Tools.FieldNavigation;
@TeleOp
public class NaviTest extends BaseTeleOp {
    BaseHardwareMap robot;
    FieldNavigation navi;
    GyroHardwareMap gyro;

    double wy;

    @Override
    public void initialize() {
        robot = new FullHardwareMap(hardwareMap);
        gyro = new GyroHardwareMap(hardwareMap);
        navi = new FieldNavigation(robot, gyro, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    @Override
    public void loop() {

        if (!navi.drive) {
            if (gamepad1.a) {
                navi.drive_to_pos(0,0,0.5,2);
            }
            else if (gamepad1.b) {
                navi.drive_to_pos(0,100,0.5,2);
            }
            else if (gamepad1.y) {
                navi.drive_to_pos(100,100,0.5,2);
            }
            else if (gamepad1.x) {
                navi.drive_to_pos(100, 0, 0.5, 2);
            }
        }

        navi.step();
        telemetry.addData("Pos x", navi.position_x);
        telemetry.addData("Pos z", navi.position_z);
        telemetry.addData("rotY", navi.rotation_y);
        telemetry.addData("wy", navi.wy);
        telemetry.addData("vx", navi.vx);
        telemetry.addData("vz", navi.vz);
        telemetry.addData("Drive", navi.drive);
        telemetry.update();

    }


}
