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
            if (robot.motor_lift.getCurrentPosition() < lift_start_encoder_value - 6000) // slow down if the lift is up
                speed = 0.18;
            else
                speed = 0.25;
            navi.drive_setSpeed(gamepad2.left_stick_y,gamepad2.left_stick_x, gamepad2.left_trigger!=0.0?-gamepad2.left_trigger:gamepad2.right_trigger,speed);
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
            if (!gamepad1.dpad_down && robot.motor_lift.getCurrentPosition() <= lift_start_encoder_value && robot.motor_lift.getCurrentPosition() >= lift_start_encoder_value - 10200 )
            {
                if (robot.motor_lift.getMode() == DcMotor.RunMode.RUN_TO_POSITION)
                    robot.motor_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                // set motor lift power
                robot.motor_lift.setPower(gamepad2.right_stick_y);
            }
        }
        // lift reset mode (to make the lift hold its position)
        else if (robot.motor_lift.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
            if(gamepad2.left_bumper && gamepad1.dpad_up){
                lift_start_encoder_value = robot.motor_lift.getCurrentPosition();
            }
            if (!gamepad1.dpad_up && robot.motor_lift.getCurrentPosition() < lift_start_encoder_value - 10200){
                robot.motor_lift.setTargetPosition(lift_start_encoder_value - 10200);
            }
            else if (!gamepad1.dpad_up && robot.motor_lift.getCurrentPosition() > lift_start_encoder_value){
                robot.motor_lift.setTargetPosition(lift_start_encoder_value);
            }
            else {
                robot.motor_lift.setTargetPosition(robot.motor_lift.getCurrentPosition());
            }
            robot.motor_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motor_lift.setPower(1.0);
        }

        /* automate lift heights
         */
        if (gamepad2.a) { // down
            if (robot.motor_lift.getCurrentPosition() < lift_start_encoder_value-3000) {
                robot.servo1.setPosition(0.4);
                robot.servo2.setPosition(0.0);
            }
            robot.servo3.setPosition(0.0);
            robot.motor_lift.setTargetPosition(lift_start_encoder_value);
        } else if (gamepad2.b) { // low
            robot.servo3.setPosition(0.3);
            robot.motor_lift.setTargetPosition(lift_start_encoder_value - 3850);
        } else if (gamepad2.x) { // mid
            robot.servo3.setPosition(0.3);
            robot.motor_lift.setTargetPosition(lift_start_encoder_value - 6950);
        } else if (gamepad2.y) { // high
            robot.servo3.setPosition(0.3);
            robot.motor_lift.setTargetPosition(lift_start_encoder_value - 10000);
        }

        // driving (gamepad1)
        if (!disable_gamepad1) {
            // change max speed
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
        } // reset drive speed
        else if (gamepad2.left_stick_x == 0 && gamepad2.left_stick_y == 0 && gamepad2.left_trigger == 0 && gamepad2.right_trigger == 0)
            navi.drive_setSpeed(0,0,0,0);

        /* gamepad2 open/close claw
         *
         */
        if (gamepad2.dpad_up) {
            robot.servo3.setPosition(0.3);
        } else if (gamepad2.dpad_down) {
            robot.servo4.setPosition(0.16);
            robot.servo3.setPosition(0.0);
        } else if (gamepad2.dpad_left) {
            robot.servo1.setPosition(0.0);
            robot.servo2.setPosition(0.4);
        } else if (gamepad2.dpad_right) {
            robot.servo1.setPosition(0.4);
            robot.servo2.setPosition(0.0);
        }

        // servo 4
        if (robot.motor_lift.getCurrentPosition() >= lift_start_encoder_value-500)
            robot.servo4.setPosition(0.0);
        else if (robot.servo3.getPosition() == 0.0 || robot.servo1.getPosition() == 0.0)
            robot.servo4.setPosition(0.16);
        else if (robot.servo4.getPosition() != 0.42)
            robot.servo4.setPosition(0.42);

        telemetry.addData("lift pos :", "%d (%s)", robot.motor_lift.getCurrentPosition(), (robot.motor_lift.getMode()==DcMotor.RunMode.RUN_TO_POSITION)?"POS":"ENC");
        telemetry.addData("claw status :", (robot.servo1.getPosition() == 0)? "opened":"closed");
        telemetry.addData("claw status :", (robot.servo3.getPosition() == 0)? "down":"up");

        navi.step();
        telemetry.update();
    }
}
