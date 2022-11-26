package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Date;

@Autonomous
public class AutoRight extends BaseAutonomous {
    @Override
    public void run() {
        /*
        driveToJunctionMid();
        placeConeOnMid();
        parkTerminal();
        */

        /*
        navi.drive_rel(120,0,0.4,1);
        while (navi.drive && opModeIsActive()) {
            navi.step();
        }
        // place
        navi.drive_rel(-120,0,0.4,1);
        while (navi.drive && opModeIsActive()) {
            navi.step();
        }
        navi.drive_rel(0,200,0.4,1);
        while (navi.drive && opModeIsActive()) {
            navi.step();
        }
        */
        robot.servo1.setPosition(0.4);
        robot.servo2.setPosition(0.0);
        long startTime = (new Date()).getTime();
        while (startTime+1000 > (new Date()).getTime() && opModeIsActive()) {
        }
        robot.servo3.setPosition(0.3);
        robot.motor_lift.setTargetPosition((int) lift_start_pos-6850);
        robot.motor_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        navi.drive_to_pos(-70, 0, 0.35, 0.5);
        startTime = (new Date()).getTime();
        while (startTime+1300 > (new Date()).getTime() && opModeIsActive() && navi.drive) {
            navi.step();
        }
        robot.motor_lift.setPower(0.7);
        while (navi.drive && opModeIsActive()) {
            navi.step();
        }
        /*
        navi.drive_setMotors(0,-1,0,0.5);
        while(navi.position_z > -60 && opModeIsActive()) {
            navi.stepPos();
        }
         */
        navi.drive_setMotors(0,0,-1,0.5);
        while(navi.rotation_y > -35 && opModeIsActive()) {
            navi.stepRotation();
            navi.stepPos();
        }
        navi.drive_setMotors(-1,0,0,0.3);
        startTime = (new Date()).getTime();
        while (startTime+550 > (new Date()).getTime() && opModeIsActive()) {
            navi.stepPos();
        }
        navi.drive_setMotors(0,0,0,0);
        while (robot.motor_lift.getCurrentPosition() != lift_start_pos-6850 && opModeIsActive()) {
        }
        robot.servo3.setPosition(0.1);
        startTime = (new Date()).getTime();
        while (startTime+200 > (new Date()).getTime() && opModeIsActive()) {
        }
        robot.servo1.setPosition(0.0);
        robot.servo2.setPosition(0.4);
        while (startTime+500 > (new Date()).getTime() && opModeIsActive()) {
        }
        robot.motor_lift.setTargetPosition((int) lift_start_pos);
        while (robot.motor_lift.getCurrentPosition() != lift_start_pos && opModeIsActive()) {
        }
        robot.motor_lift.setPower(0);
        robot.motor_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        navi.drive_setMotors(1,0,0,0.7);
        startTime = (new Date()).getTime();
        while (startTime+2000 > (new Date()).getTime() && opModeIsActive()) {
        }
        navi.drive_setMotors(0,0,0,0);
    }
}
