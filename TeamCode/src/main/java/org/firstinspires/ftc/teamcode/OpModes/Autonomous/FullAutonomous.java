package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

public class FullAutonomous extends BaseAutonomous {
    @Override
    public void run() {
        detectSignal();
        driveToJunctionHigh();
        driveToZone();
    }
};
