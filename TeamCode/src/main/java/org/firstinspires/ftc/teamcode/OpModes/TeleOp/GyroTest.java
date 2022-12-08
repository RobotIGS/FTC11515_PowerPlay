package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.HardwareMaps.GyroHardwareMap;

@TeleOp
public class GyroTest extends BaseTeleOp {
    GyroHardwareMap gyro;
    Orientation rot;

    public void initialize() {
        gyro = new GyroHardwareMap(hardwareMap);
    }

    public void loop() {
        rot = gyro.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Z", rot.firstAngle);
        telemetry.addLine();
        telemetry.addData("X", rot.secondAngle);
        telemetry.addData("Y", rot.thirdAngle);
    }
}
