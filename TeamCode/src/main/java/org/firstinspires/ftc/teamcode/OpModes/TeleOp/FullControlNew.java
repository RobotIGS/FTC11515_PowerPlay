package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HardwareMaps.BaseHardwareMap;
import org.firstinspires.ftc.teamcode.HardwareMaps.FullHardwareMap;
import org.firstinspires.ftc.teamcode.HardwareMaps.GyroHardwareMap;
import org.firstinspires.ftc.teamcode.Tools.FieldNavigation;

@TeleOp
public class FullControlNew extends BaseTeleOp {
    BaseHardwareMap robot;
    FieldNavigation navi;
    GyroHardwareMap gyro;

    public double lift_start_encoder_value;
    public boolean disable_gamepad1;

    double wy;
    double speed;

    @Override
    public void initialize() {
        robot = new FullHardwareMap(hardwareMap);
        gyro = new GyroHardwareMap(hardwareMap);
        navi = new FieldNavigation(robot, gyro, 0.0, 0.0, 0.0, 0.0/180, 0.0);
        lift_start_encoder_value  = robot.motor_lift.getCurrentPosition();
    }
    

    @Override
    public void loop() {
        /* gamepad2 drive overwrite
         * gamepad1: slave
         * gamepad2: master
         *   - drive slow
         */
        if (gamepad2.left_stick_y != 0.0 && gamepad2.left_stick_x != 0.0) {
            speed = 0.25;
            navi.drive_setSpeed(gamepad2.left_stick_y,gamepad2.left_stick_x,gamepad2.right_stick_x*0.5, speed);
            disable_gamepad1 = true;
        }

        /* gampead2 manual lift control
         *   gamepad1: master
         *      - drive normal / slow (dep on lift height)
         *   gamepad2: slave
         */
        if (gamepad2.right_stick_y != 0) {
            // set lift motor mode
            if (robot.motor_lift.getMode() == DcMotor.RunMode.RUN_TO_POSITION)
                robot.motor_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // set motor lift power
            robot.motor_lift.setPower(gamepad2.right_stick_y);
        }
        // lift reset mode
        else if (robot.motor_lift.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
            robot.motor_lift.setTargetPosition(robot.motor_lift.getCurrentPosition());
            robot.motor_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motor_lift.setPower(1);
        }

        if (!disable_gamepad1) {
            if (
                    gamepad1.right_trigger != 0.0 &&
                    robot.motor_lift.getCurrentPosition() >= lift_start_encoder_value - 4800
            )
                speed = 0.5 + gamepad1.right_trigger * 0.25;
            else if (gamepad1.left_trigger != 0.0)
                speed = 0.5 - gamepad1.left_trigger * 0.25;
            else
                speed = 0.5;
            navi.drive_setSpeed(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x*0.5, speed);
        }
        if (gamepad1.a) {
            robot.motor_lift.setTargetPosition((int) lift_start_encoder_value - 5);
            // -5
        } else if (gamepad1.b) {
            robot.servo3.setPosition(0.3);
            robot.motor_lift.setTargetPosition((int) lift_start_encoder_value - 3850);
            // -5100
        } else if (gamepad1.x) {
            robot.servo3.setPosition(0.3);
            robot.motor_lift.setTargetPosition((int) lift_start_encoder_value - 6950);
            // -8050
        } else if (gamepad1.y) {
            robot.servo3.setPosition(0.3);
            robot.motor_lift.setTargetPosition((int) lift_start_encoder_value - 10400);
            robot.servo4.setPosition(1.6);
            // -10400

        telemetry.addData("lift pos :", robot.motor_lift.getCurrentPosition());
        telemetry.addData("claw status :", (robot.servo1.getPosition() == 0)? "opened":"closed");

        }

        if (gamepad2.dpad_left) {
            robot.servo1.setPosition(0.0);
            robot.servo2.setPosition(0.4);
        } else if (gamepad2.dpad_right) {
            robot.servo1.setPosition(0.4);
            robot.servo2.setPosition(0.0);
            }

        navi.step();
        telemetry.update();
    }
}
