package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class SimpleAutonomous extends BaseAutonomous {
    @Override
    public void run() {
        /*
        driveToJunctionMid();
        placeConeOnMid();
        parkTerminal();
        */

        navi.drive_rel(120,0,0.2,1);
        while (navi.drive && opModeIsActive()) {
            navi.step();
        }
        navi.drive_rel(-120,0,0.2,1);
        while (navi.drive && opModeIsActive()) {
            navi.step();
        }
        navi.drive_rel(0,200,0.2,1);
        while (navi.drive && opModeIsActive()) {
            navi.step();
        }
    }
}
