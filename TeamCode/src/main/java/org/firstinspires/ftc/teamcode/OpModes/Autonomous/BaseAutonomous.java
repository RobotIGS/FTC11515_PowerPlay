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

        /*
        [-108,0,30] (starting position)
        [-108,0,0]
        aQW
             (three points?)
             [-78,0,0]
             [-108,0,90]
             [-108,0,150]

        [-78,0,0]
        [0,0,0]
         */



    };

    public void driveToZone() {

    };

    public abstract BaseHardwareMap initializeHardwareMap();
    
    public abstract void run();
}
