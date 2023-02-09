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

    public int lift_start_encoder_value;
    public boolean disable_gamepad1 = false;
    double speed;

    @Override
    public void initialize() {
        robot = new FullHardwareMap(hardwareMap);
        gyro = new GyroHardwareMap(hardwareMap);
        navi = new FieldNavigation(robot, gyro, 0.0, 0.0, 0.0, 0.0/180, 0.0);
        // motor lift init stuff
        lift_start_encoder_value  = robot.motor_lift.getCurrentPosition();
        robot.motor_lift.setTargetPosition(lift_start_encoder_value);
        robot.motor_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor_lift.setPower(1.0);
    }
    

    @Override
    public void loop() {
        /* gamepad2 drive overwrite
         * gamepad1: slave
         * gamepad2: master
         *   - drive slow
         */
        if (gamepad2.left_stick_y != 0.0 || gamepad2.left_stick_x != 0.0 || gamepad2.left_trigger != 0.0 || gamepad2.right_trigger != 0.0) {
            navi.drive_setSpeed(gamepad2.left_stick_y,gamepad2.left_stick_x, gamepad2.left_trigger!=0.0?-gamepad2.left_trigger:gamepad2.right_trigger,0.25);
            disable_gamepad1 = true;
        } else {
            disable_gamepad1 = false;
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
            robot.motor_lift.setPower(1.0);
        }

        if (gamepad2.a) {
            if (robot.motor_lift.getCurrentPosition() < lift_start_encoder_value-3000) {
                robot.servo1.setPosition(0.4);
                robot.servo2.setPosition(0.0);
            }
            robot.servo3.setPosition(0.0);
            robot.motor_lift.setTargetPosition(lift_start_encoder_value);
        } else if (gamepad2.b) {
            robot.servo3.setPosition(0.3);
            robot.motor_lift.setTargetPosition(lift_start_encoder_value - 3850);
        } else if (gamepad2.x) {
            robot.servo3.setPosition(0.3);
            robot.motor_lift.setTargetPosition(lift_start_encoder_value - 6950);
        } else if (gamepad2.y) {
            robot.servo3.setPosition(0.3);
            robot.motor_lift.setTargetPosition(lift_start_encoder_value - 10000);
        }

        if (!disable_gamepad1) {
            if (gamepad1.right_trigger != 0.0 && robot.motor_lift.getCurrentPosition() >= lift_start_encoder_value - 4800)
                speed = 0.5 + gamepad1.right_trigger * 0.25;
            else if (gamepad1.left_trigger != 0.0)
                speed = 0.5 - gamepad1.left_trigger * 0.25;
            else if (robot.motor_lift.getCurrentPosition() <= lift_start_encoder_value-6500)
                speed = 0.25;
            else
                speed = 0.5;
            // auto lift height when fast/norm
            if ((gamepad1.left_stick_y!=0.0 || gamepad1.left_stick_x!=0.0 && speed >= 0.5) && Math.max(robot.motor_lift.getCurrentPosition(), robot.motor_lift.getTargetPosition()) >= lift_start_encoder_value - 500)
                robot.motor_lift.setTargetPosition(lift_start_encoder_value - 550);
            navi.drive_setSpeed(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x*0.5, speed);
        } // reset drive
        else if (gamepad2.left_stick_x == 0 && gamepad2.left_stick_y == 0 && gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0)
            navi.drive_setSpeed(0,0,0,0);

        /* gamepad2 open/close claw
         *
         */
        if (gamepad2.dpad_left) {
            robot.servo1.setPosition(0.0);
            robot.servo2.setPosition(0.4);
        } else if (gamepad2.dpad_right) {
            robot.servo1.setPosition(0.4);
            robot.servo2.setPosition(0.0);
        }
        if (gamepad2.dpad_up) {
            robot.servo3.setPosition(0.3);
        } else if (gamepad2.dpad_down) {
            robot.servo3.setPosition(0.0);
        }

        telemetry.addData("lift pos :", "%d (%s)", robot.motor_lift.getCurrentPosition(), (robot.motor_lift.getMode()==DcMotor.RunMode.RUN_TO_POSITION)?"POS":"ENC");
        telemetry.addData("claw status :", (robot.servo1.getPosition() == 0)? "opened":"closed");
        telemetry.addData("claw status :", (robot.servo3.getPosition() == 0)? "down":"up");

        navi.step();
        telemetry.update();
    }
}
