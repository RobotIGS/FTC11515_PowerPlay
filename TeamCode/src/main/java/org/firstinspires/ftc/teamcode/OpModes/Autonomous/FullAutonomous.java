package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

public class FullAutonomous extends BaseAutonomous {
    @Override
    protected void initialize() {
        super.initialize();
        initVuforia();
        initTFod();
    }
    @Override
    public void run() {
        detectSignal();
        driveToJunctionHigh();
        driveToZone();
    }
};
