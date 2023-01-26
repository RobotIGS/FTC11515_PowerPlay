package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class TestSignalDetection extends BaseAutonomous{

    @Override
    public void run() {
        telemetry.addLine("starting...");
        telemetry.update();
        detectSignal();
        telemetry.addData("signal", signal_detected);
    }
}
