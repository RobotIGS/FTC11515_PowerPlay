package org.firstinspires.ftc.teamcode.Tools;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.HardwareMaps.BaseHardwareMap;
import org.firstinspires.ftc.teamcode.HardwareMaps.GyroHardwareMap;

public class FieldNavigation {
    private final BaseHardwareMap robot;
    private final GyroHardwareMap gyro;
    private final PIController gyro_pi_controller;

    // constants
    private final double MOTOR_STEPS_PER_REV = 751.8; // 223rpm:751.8
    private final double WHEEL_DIAMETER_CM = 10;

    // finals
    private final double MOTOR_STEPS_PER_CM = MOTOR_STEPS_PER_REV / (Math.PI*WHEEL_DIAMETER_CM);
    protected final double ONE_D_R = 1 / (WHEEL_DIAMETER_CM/2);
    protected final double R_D_FOUR = WHEEL_DIAMETER_CM/8;
    protected final double TWOPI_D_CMPERREV = 2*Math.PI / MOTOR_STEPS_PER_REV;

    // robot start positioning
    protected final double start_position_x;
    protected final double start_position_z;
    protected final double start_rotation_y;
    protected final double start_gyro_rotation;

    // robot positioning
    public double position_x;
    public double position_z;
    public double rotation_y;

    // moving speeds
    public double speed_vx;
    public double speed_vz;
    public double speed_wy;

    // driving flags
    private boolean is_driving;
    private boolean is_target_reached;

    // driving targets
    private double target_position_x;
    private double target_position_z;
    private double target_rotation_y;

    // driving speeds
    private double driving_speed;
    private double target_driving_speed;

    // driving accuracy
    private double drive_acc;

    // navigation stuff
    private double[] motor_steps_last = new double[4];
    private double[] motor_gyro_correction_steps = new double[4];

    public FieldNavigation(
            BaseHardwareMap robot, GyroHardwareMap gyro, PIController gyro_pi_controller,
            double position_x, double position_z, double rotation_y) {
        this.robot = robot;
        this.gyro = gyro;
        this.gyro_pi_controller = gyro_pi_controller;

        // start values (finals)
        start_position_x = position_x;
        start_position_z = position_z;
        start_rotation_y = rotation_y;

        // getting start motor rotation
        motor_steps_last[0] = robot.motor_front_left.getCurrentPosition();
        motor_steps_last[1] = robot.motor_front_right.getCurrentPosition();
        motor_steps_last[2] = robot.motor_rear_left.getCurrentPosition();
        motor_steps_last[3] = robot.motor_rear_right.getCurrentPosition();

        // getting gyro start rotation
        start_gyro_rotation = gyro.imu.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        // setting positions
        this.position_x = position_x;
        this.position_z = position_z;
        this.rotation_y = rotation_y;
    }

    /**
     * Calculates the speeds for each driving motor.
     * @param vx speed in x direction
     * @param vz speed in z direction
     * @param wy rotation speed (y axes)
     * @return array with motor speeds
     */
    protected double[] CalculateWheelSpeeds(double vx, double vz, double wy) {
        return new double[] {
                (vx+vz-wy) * ONE_D_R,
                (vx-vz+wy) * ONE_D_R,
                (vx-vz-wy) * ONE_D_R,
                (vx+vz+wy) * ONE_D_R,
        };
    }

    /**
     * Setting the drive accuracy.
     * @param accuracy accuracy in CM
     */
    public void Drive_SetTargetAccuracy(double accuracy) {
        drive_acc = accuracy;
    }

    /**
     * Drive to Field position.
     * @param x coordinate
     * @param z coordinate
     * @param speed max driving speed
     */
    public void Drive_ToPos(double x, double z, double speed) {
        target_position_x = x;
        target_position_z = z;
        target_driving_speed = speed;
    }

    /**
     * Drive a distance relative to the robot.
     * @param x distance in x
     * @param z distance in z
     * @param speed max driving speed
     */
    public void Drive_RelPos(double x, double z, double speed) {
        // convert x,z to real coordinates
        Drive_ToPos(x,z,speed);
    }

    /**
     * Setting motor speeds based on direction speeds.
     * @param vx speed in x direction
     * @param vz speed in z direction
     * @param wy rotation speed y axes
     * @param speed speed factor applied to each motor speed.
     */
    protected void Drive_setMotorSpeeds(double vx, double vz, double wy, double speed) {
        double[] wheelSpeeds = CalculateWheelSpeeds(vx,vz,wy);

        robot.motor_front_left.setPower(wheelSpeeds[0]*speed);
        robot.motor_front_right.setPower(wheelSpeeds[1]*speed);
        robot.motor_rear_left.setPower(wheelSpeeds[2]*speed);
        robot.motor_rear_right.setPower(wheelSpeeds[3]*speed);
    }

    public void Drive_Stop() {
        is_driving = false;
    }

    public boolean IsTargetReached() {
        // test if the robot already reached the target position
        if (!is_target_reached) {
            // calculate distance to target position
            double d = Math.sqrt(
                    Math.pow(target_position_x - position_x, 2) +
                    Math.pow(target_position_z - position_z, 2));

            // test if the robot is in a radius of the target (r=drive_acc)
            if (d <= drive_acc) {
                // setting target reached and stop the robot
                is_target_reached = true;
                Drive_Stop();
            }
        }

        // return if the robot has reached the target position
        return is_target_reached;
    }

    public boolean IsDriving() {
        return is_driving;
    }

    /**
     * Update robot position based on motor movement (steps).
     */
    public void step_Position_motorSteps() {
        // getting delta movement of motors
        double[] deltaSteps = {
                motor_steps_last[0] - robot.motor_front_left.getCurrentPosition(),
                motor_steps_last[1] - robot.motor_front_right.getCurrentPosition(),
                motor_steps_last[2] - robot.motor_rear_left.getCurrentPosition(),
                motor_steps_last[3] - robot.motor_rear_left.getCurrentPosition(),
        };

        // update remembered motor steps.
        motor_steps_last[0] -= deltaSteps[0];
        motor_steps_last[1] -= deltaSteps[1];
        motor_steps_last[2] -= deltaSteps[2];
        motor_steps_last[3] -= deltaSteps[3];

        // subtract steps from rotation correction
        deltaSteps[0] -= motor_gyro_correction_steps[0];
        deltaSteps[1] -= motor_gyro_correction_steps[1];
        deltaSteps[2] -= motor_gyro_correction_steps[2];
        deltaSteps[3] -= motor_gyro_correction_steps[3];

        // resetting the gyro correction steps
        motor_gyro_correction_steps[0] = 0;
        motor_gyro_correction_steps[1] = 0;
        motor_gyro_correction_steps[2] = 0;
        motor_gyro_correction_steps[3] = 0;

        // getting the relative movement
        double[] deltaMovement = new double[2];

        deltaMovement[0] = (deltaSteps[0]+deltaSteps[1]+deltaSteps[2]+deltaSteps[3]) *
                R_D_FOUR * TWOPI_D_CMPERREV;
        deltaMovement[1] = (deltaSteps[0]-deltaSteps[1]-deltaSteps[2]+deltaSteps[3]) *
                R_D_FOUR * TWOPI_D_CMPERREV;

        // converting relative movement to absolute movement on the field
        // TODO: deltaMovement = convert...(...[0],...[1]);

        // applying the delta movement
        position_x += deltaMovement[0];
        position_z += deltaMovement[1];
    }

    public void step_Rotation() {
        // getting rotation
        rotation_y = start_gyro_rotation - gyro.imu.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + start_rotation_y;

        // normalize rotation
        if (rotation_y < -180)
            rotation_y = 180 -(rotation_y%180);
        else if (rotation_y > 180)
            rotation_y = -180 +(rotation_y%180);
    }

    public void step_RotationCorrection() {
        // getting the rotation speed base on error
        gyro_pi_controller.step(target_rotation_y-rotation_y);
        // TODO add normalization
        speed_wy = gyro_pi_controller.out;

        // set the gyro correction steps
        motor_gyro_correction_steps = CalculateWheelSpeeds(0,0,speed_wy);
    }

    public void step_Drive() {
        // overwrites speeds before use if the robot is driving autonomous
        if (is_driving && !IsTargetReached()) {
            double[] speeds = new double[2];
            // set speed in direction of the target.
            speeds[0] = target_position_x - position_x;
            speeds[1] = target_position_z - position_z;

            // transform pos to rel
            // TODO pos2rel(speeds)
            speed_vx = speeds[0];
            speed_vz = speeds[1];
        }
        Drive_setMotorSpeeds(speed_vx, speed_vz, speed_wy, driving_speed);
    }

    public void step_Speed() {
        // TODO
    }

    public void step() {
        // TODO
    }
}
