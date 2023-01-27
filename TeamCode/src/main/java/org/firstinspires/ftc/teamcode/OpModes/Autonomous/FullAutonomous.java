package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

public class FullAutonomous extends BaseAutonomous {
    @Override
    protected void initialize() {
        super.initialize();
        initVuforia();
        initTFod();
        telemetry.addLine("init: Done");
        telemetry.update();
    }
    @Override
    public void run() {
        driveToJunctionHigh();
        driveToZone();
        navi.drive_setMotors(0,0,0,0);
    }
};
