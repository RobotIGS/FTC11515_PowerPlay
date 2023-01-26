package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class FullAutonomous extends BaseAutonomous {
    @Override
    protected void initialize() {
        super.initialize();
        //initVuforia();
        //initTFod();
    }
    @Override
    public void run() {
        //detectSignal();
        signal_detected = 0;
        driveToJunctionHigh();
        driveToZone();
    }
};
