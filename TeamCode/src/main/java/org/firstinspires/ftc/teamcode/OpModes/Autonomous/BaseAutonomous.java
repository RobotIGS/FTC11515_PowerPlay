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

    void initialize() {
        robot = new FullHardwareMap(hardwareMap);
        navi = new FieldNavigation(robot,gyro,0.0,0.0,0.0,0.,0.0);
    }

    public void detectSignal() {
    };

    public void driveToJunction() {
        navi.drive_to_pos(0.0,147.0,0.2,1.0);
        navi.drive_to_pos(0.0,90.0,0.2,1.0);

        /*
        [-108,0,30] (starting position)
        [-108,0,0]
        aQW
             (three points?)
             [-78,0,0]
             [-108,0,90]
             [-108,0,150]

        [-78,0,0]
        [0,0,0]
         */



    };

    public void driveToZone() {
        navi.drive_to_pos(-29.0,90.0,0.2,1.0);
        navi.drive_to_pos(-88.0,90.0,0.2,1.0);
        navi.drive_to_pos(-149.0,90.0,0.2,1.0);
    };

    public abstract BaseHardwareMap initializeHardwareMap();
    
    public abstract void run();
}
