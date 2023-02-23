package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

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
        cv_signal_detector.detect();
        driveToJunctionHigh();
        driveToZone();
        navi.drive_setMotors(0,0,0,0);
    }
};
