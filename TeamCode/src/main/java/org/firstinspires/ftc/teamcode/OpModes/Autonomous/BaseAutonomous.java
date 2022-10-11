package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.HardwareMaps.BaseHardwareMap;

public abstract class BaseAutonomous extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        run();
    }

    void initialize() {
    }

    public void detectSignal() {
    };

    public void driveToJunction() {

    };

    public void driveToZone() {

    };

    public abstract BaseHardwareMap initializeHardwareMap();
    
    public abstract void run();
}
