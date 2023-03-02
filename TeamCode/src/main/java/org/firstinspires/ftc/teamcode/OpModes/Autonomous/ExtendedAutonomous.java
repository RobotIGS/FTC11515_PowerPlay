package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

public class ExtendedAutonomous extends BaseAutonomous {
    @Override
    public void detectSignal() {}
    @Override
    public void run() {
        signal_detected = 0;
        initDriveAfterStart();
        driveToJunctionHigh();
        driveToZone();
    }
}
