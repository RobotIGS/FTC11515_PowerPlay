package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareMaps.BaseHardwareMap;
import org.firstinspires.ftc.teamcode.HardwareMaps.FullHardwareMap;
import org.firstinspires.ftc.teamcode.HardwareMaps.GyroHardwareMap;
import org.firstinspires.ftc.teamcode.Tools.FieldNavigation;

public abstract class BaseAutonomous extends LinearOpMode {
    private BaseHardwareMap robot;
    private GyroHardwareMap gyro;
    private FieldNavigation navi;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        run();
    }

    private void initialize() {
        robot = new FullHardwareMap(hardwareMap);
        gyro = new GyroHardwareMap(hardwareMap);
        navi = new FieldNavigation(robot, gyro,
                Quadrant() % 2 == 0 ? -89 : 89,
                Quadrant() / 2 >= 1 ? -155 : 155,
                Quadrant() < 2 ? 90 : -90,
                0.7/180., 0.5
        );
    }

    public int Quadrant() {
        return 0;
    }

    private void driveToPosQ(double x, double z, double speed, double acc) {
        if (Quadrant() % 2 == 0) {
            x = -x;
        } if (Quadrant() / 2 >= 1) {
            z = -z;
        }
        navi.drive_to_pos(x, z, speed, acc);
        while (navi.drive && opModeIsActive()) {
            telemetry.addData("x", navi.position_x);
            telemetry.addData("z", navi.position_z);
            telemetry.update();
            navi.step();
        }
    }

    public void detectSignal() {
    }

    public void driveToJunctionMid() {
        driveToPosQ(76, 72, 0.2, 0.3);
    }

    protected void placeConeOnMid() {
        // lift up
        navi.set_targetRotation((Quadrant() == 0 || Quadrant() == 3) ? navi.rotation_y+45 : navi.rotation_y-45);
        while (Math.abs(navi.wy) >= 0.06) {
            navi.stepRotate();
        }
        // lift down
    }

    protected void parkTerminal() {
        navi.drive_rel(-150,0,0.2,2);
        while (navi.drive && opModeIsActive()) {
            navi.step();
        }
    }

    public void driveToJunctionHigh() {
        navi.drive_to_pos(0.0, 147.0, 0.2, 0.3);
        while (navi.drive && opModeIsActive()) {
            navi.step();
        }

        navi.drive_to_pos(0.0, 90.0, 0.2, 0.3);
        while (navi.drive && opModeIsActive()) {
            navi.step();
        }
    }

    public void driveToZone() {
        navi.drive_to_pos(-29.0,90.0,0.2,0.3);
        while (navi.drive && opModeIsActive()) {
            navi.step();
        }

        navi.drive_to_pos(-88.0,90.0,0.2,0.3);
        while (navi.drive && opModeIsActive()) {
            navi.step();
        }

        navi.drive_to_pos(-149.0,90.0,0.2,0.3);
        while (navi.drive && opModeIsActive()) {
            navi.step();
        }
    }

    public abstract void run();
}
