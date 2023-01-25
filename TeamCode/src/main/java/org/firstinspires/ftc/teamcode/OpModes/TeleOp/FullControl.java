package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HardwareMaps.BaseHardwareMap;
import org.firstinspires.ftc.teamcode.HardwareMaps.FullHardwareMap;
import org.firstinspires.ftc.teamcode.HardwareMaps.GyroHardwareMap;
import org.firstinspires.ftc.teamcode.Tools.FieldNavigation;

@TeleOp
public class FullControl extends BaseTeleOp {
    BaseHardwareMap robot;
    FieldNavigation navi;
    GyroHardwareMap gyro;
    double servoPos = 0;


    public double lift_start_encoder_value;

    double wy;
    public boolean f = false;

    @Override
    public void initialize() {

        robot = new FullHardwareMap(hardwareMap);
        gyro = new GyroHardwareMap(hardwareMap);
        navi = new FieldNavigation(robot, gyro, 0.0, 0.0, 0.0, 0.0/180, 0.0);
        lift_start_encoder_value  = robot.motor_lift.getCurrentPosition();
    }

    @Override
    public void loop() {
        if (f == true && !robot.motor_lift.isBusy()){
            robot.servo3.setPosition(0.1);
        }
        if (!gamepad2.left_bumper) {
            if (gamepad1.right_stick_y != 0) {
                double lift_pos = robot.motor_lift.getCurrentPosition() - lift_start_encoder_value;
                if (lift_pos > -10400.0 && lift_pos < -1) {
                    if (robot.motor_lift.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                        robot.motor_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                    robot.motor_lift.setPower(gamepad1.right_stick_y);
                } else {
                    if (lift_pos > -10) {
                        robot.motor_lift.setTargetPosition((int) lift_start_encoder_value - 5);
                        robot.motor_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.motor_lift.setPower(0.5);
                    } else if (lift_pos < -10000) {
                        robot.motor_lift.setTargetPosition((int) lift_start_encoder_value - 10400);
                        robot.motor_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.motor_lift.setPower(0.5);
                    }
                }
            } else if (gamepad1.a) {
                f = true;
                robot.motor_lift.setTargetPosition((int) lift_start_encoder_value - 5);
                robot.motor_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.motor_lift.setPower(1);
                // -5
            } else if (gamepad1.b) {
                robot.servo3.setPosition(0.3);
                robot.motor_lift.setTargetPosition((int) lift_start_encoder_value - 3750);
                robot.motor_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.motor_lift.setPower(1);
                // -5100
            } else if (gamepad1.x) {
                robot.servo3.setPosition(0.3);
                robot.motor_lift.setTargetPosition((int) lift_start_encoder_value - 6850);
                robot.motor_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.motor_lift.setPower(1);
                // -8050
            } else if (gamepad1.y) {
                robot.servo3.setPosition(0.3);
                robot.motor_lift.setTargetPosition((int) lift_start_encoder_value - 10100);
                robot.motor_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.motor_lift.setPower(1);
                // -10400
            } else if (robot.motor_lift.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                robot.motor_lift.setPower(0);
            }
        } else {
            if (gamepad2.right_stick_y != 0) {
                robot.motor_lift.setPower(gamepad2.right_stick_y);
            } else {
                robot.motor_lift.setPower(0);
                if (gamepad2.a) {
                    lift_start_encoder_value = robot.motor_lift.getCurrentPosition();
                }
            }
        }
        if (gamepad1.dpad_left) {
            robot.servo1.setPosition(0.0);
            robot.servo2.setPosition(0.4);
        } else if (gamepad1.dpad_right) {
            robot.servo1.setPosition(0.4);
            robot.servo2.setPosition(0.0);
        }
        if (gamepad1.dpad_down) {
            robot.servo3.setPosition(0.1);
        }
        if (gamepad1.dpad_up)  {
            robot.servo3.setPosition(0.3);
        }
        wy = (gamepad1.left_trigger != 0.0) ? -gamepad1.left_trigger : gamepad1.right_trigger;

        navi.drive_setSpeed(gamepad1.left_stick_y,gamepad1.left_stick_x,wy*0.5,(gamepad1.left_bumper || gamepad1.right_bumper) ? 0.75 : 0.5);
        navi.step();

        // update servoPos
        if (gamepad2.right_stick_y != 0) {
            servoPos += gamepad2.right_stick_y +0.005;
        }
        // servoPos in [0;1]
        if (servoPos < 0)
            servoPos = 0;
        if (servoPos > 1)
            servoPos = 1;
        // set servo position
        robot.servo4.setPosition(servoPos);
        telemetry.addData("x :", navi.position_x);
        telemetry.addData("z :", navi.position_z);
        telemetry.addData("rotY :", navi.rotation_y);
        telemetry.addData("lift :", robot.motor_lift.getCurrentPosition()-lift_start_encoder_value);
        telemetry.addData("servo:", robot.servo4.getPosition());
        telemetry.update();
    }


}
