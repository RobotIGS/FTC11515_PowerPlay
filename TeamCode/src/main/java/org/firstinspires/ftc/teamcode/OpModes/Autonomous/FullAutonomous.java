package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class FullAutonomous extends BaseAutonomous {
    @Override
    public void run() {

        driveToJunctionHigh();
        signal_detected = 1;
        driveToZone();
    }
};
