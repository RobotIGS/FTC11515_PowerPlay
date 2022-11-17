package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class SimpleAutonomous extends BaseAutonomous {
    @Override
    public void run() {
        telemetry.addData("step", 0);
        telemetry.update();
        driveToJunctionMid();
        telemetry.addData("step", 1);
        telemetry.update();
        placeConeOnMid();
        telemetry.addData("step", 2);
        telemetry.update();
        //parkTerminal();
    }
}
