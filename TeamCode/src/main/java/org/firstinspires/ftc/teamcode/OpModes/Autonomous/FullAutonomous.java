package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import java.util.Date;

public class FullAutonomous extends BaseAutonomous {
    @Override
    protected void initialize() {
        super.initialize();
        //initVuforia();
        //initTFod();
        initCVSignalDetection();
        telemetry.addLine("init: Done");
        telemetry.update();
    }
    @Override
    public void run() {
        detectSignalCV();
        detectSignal
        telemetry.addData("signal_detected :", signal_detected);
        telemetry.update();
        long start_time = (new Date()).getTime();
        while (start_time+5000 > (new Date()).getTime()) {
        }
        /*
        driveToJunctionHigh();
        driveToZone();
        navi.drive_setMotors(0,0,0,0);
         */
    }
};
