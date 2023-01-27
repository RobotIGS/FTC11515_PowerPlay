package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.HardwareMaps.BaseHardwareMap;
import org.firstinspires.ftc.teamcode.HardwareMaps.FullHardwareMap;
import org.firstinspires.ftc.teamcode.HardwareMaps.GyroHardwareMap;
import org.firstinspires.ftc.teamcode.HardwareMaps.WebcamHardwareMap;
import org.firstinspires.ftc.teamcode.Tools.FieldNavigation;
import java.util.Date;

import java.util.Date;
import java.util.List;
import java.util.stream.IntStream;

public abstract class BaseAutonomous extends LinearOpMode {
    private BaseHardwareMap robot;
    private GyroHardwareMap gyro;
    public FieldNavigation navi;

    private WebcamHardwareMap webcamHwMap;

    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    private static final String VUFORIA_KEY = "AfJ0TyL/////AAABmd78ofn/RkMRi5drULeQkx9J7iXzq0RVLEWyuyfXRDN3IoVgx67f+ACtVorRwa96Jnk49/2xCVBKEeei3RC9zoBnb3genq9MMD6y4kXKbyQIuFN7xispFh7+SfEtm59sNU3R5GJfTAOym68R1IU+4rgY+G4ISATIz3Y9qLBzScQDRqILmn/yGBmC2i+lw8aDepPuAND4he/bkN2ONnp5U8XBAlrZmuPWzRb63RBo5RBdWi19D3h0FOK7KgUV0sgThso9FPVRhDKqB8swS9AqcGIbMo3lqgRA/w7ON5hnRJj6RG+GV+CNDcObyiwMCtEhYaisfR6pNg1NrUTU2Cxgv6291o8fgThPYT9DNKdjz3Um";
    private static final String[] SIGNAL_LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public int signal_detected;

    public void output() {
        telemetry.addData("x",navi.position_x);
        telemetry.addData("z",navi.position_z);
        telemetry.update();
    }

    public double lift_start_encoder_value;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        run();
    }

    protected void initialize() {
        robot = new FullHardwareMap(hardwareMap);
        lift_start_encoder_value  = robot.motor_lift.getCurrentPosition();
        gyro = new GyroHardwareMap(hardwareMap);
        navi = new FieldNavigation(robot, gyro,
                Quadrant() % 2 == 0 ? 88 : -88,
                Quadrant() / 2 >= 1 ? 165 : -165,
                Quadrant() < 2 ? -90 : 90, // changed
                0.8/180., 0

        );
    }

    protected void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    public int Quadrant() {
        return 0;
    }

    private void driveToPosQ(double x, double z, double speed, double acc) {
        if (Quadrant() % 2 == 0) { // change to != 0
            x = -x;
        } if (Quadrant() / 2 >= 1) {
            z = -z;
        }
        navi.drive_to_pos(x, z, speed, acc);
        while (navi.drive && opModeIsActive()) {
            telemetry.addData("x", navi.position_x);
            telemetry.addData("z", navi.position_z);
            telemetry.addData("error ry:", navi.target_rotation_y-navi.rotation_y);
            telemetry.addData("ry:", navi.rotation_y);
            telemetry.addData("try:", navi.target_rotation_y);
            telemetry.addData("sry:", navi.start_rotation_y);
            telemetry.update();
            navi.step();
        }
    }

    public void driveToJunctionMid() {
        driveToPosQ(76, 72, 0.2, 0.3);
    }

    protected void placeConeOnMid() {
        // lift up
        navi.set_targetRotation((Quadrant() == 0 || Quadrant() == 3) ? navi.rotation_y+45 : navi.rotation_y-45);
        while (Math.abs(navi.wy) >= 0.06) {
            navi.stepRotate();
        }
        // lift down
    }

    protected void parkTerminal() {
        int fx = Quadrant()%2==0?1:-1;
        int fz = Quadrant()<2?-1:1;
        navi.drive_to_pos(-170*fx, 165*fz, 0.2, 2);
        while (navi.drive && opModeIsActive()) {
            navi.step();
        }
        navi.drive_to_pos(-170*fx, 170*fz, 0.2, 2);
        while (navi.drive && opModeIsActive()) {
            navi.step();
        }
    }

    public void driveToJunctionHigh() {
        int fx = Quadrant() % 2 == 0 ? 1:-1;
        int fz = Quadrant() < 2 ? -1:1;

        //Close Claw
        robot.servo1.setPosition(0.4);
        robot.servo2.setPosition(0.0);

        //Wait 1 second
        long startTime = (new Date()).getTime();
        while (startTime+1000 > (new Date()).getTime() && opModeIsActive()) {
        }

        //Lift Claw
        robot.servo3.setPosition(0.3);

        //Wait 1 second again
        startTime = (new Date()).getTime();
        while (startTime+1000 > (new Date()).getTime() && opModeIsActive()) {
        }



        //Lift motor arm
        robot.motor_lift.setTargetPosition((int) lift_start_encoder_value - 1000);
        robot.motor_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor_lift.setPower(1);

        while (robot.motor_lift.isBusy() && opModeIsActive()) {
        }

        robot.servo4.setPosition(0.16);

        //Wait 0.5 second
        startTime = (new Date()).getTime();
        while (startTime+500 > (new Date()).getTime() && opModeIsActive()) {
        }

        //drive forward to detect signal
        navi.drive_to_pos(navi.position_x, 120*fz,0.3,0.3);
        while (navi.drive && opModeIsActive()) {
            navi.step();
            output();
        }

        //Drive back to side line 
        navi.drive_to_pos(88.0*fx, 161.0*fz, 0.3, 0.3);
        while (navi.drive && opModeIsActive()) {
            navi.step();
            output();
        }
        startTime = (new Date()).getTime();
        while (startTime+500 > (new Date()).getTime() && opModeIsActive()) {
        }



        //Drive to x = 0
        navi.drive_to_pos(-8.0*fx,161.0*fz,0.4,0.3);
        while (navi.drive && opModeIsActive()) {
            navi.step();
            output();
        }
        //Drive against wall
        navi.drive_to_pos(-8.0*fx,187.0*fz,0.3,0.2);
        while (navi.drive && opModeIsActive()) {
            navi.step();
            output();
        }

        // reset z,x
        navi.position_z = 160*fz;
        navi.position_x = 0;

        // drive behind cone spot
        navi.drive_to_pos(0,110.0*fz,0.3,0.3);
        while (navi.drive && opModeIsActive()) {
            navi.step();
            output();
        }

        //Drive next to high junction & lift arm
        navi.drive_to_pos(0,90.0*fz,0.3,0.3);
        robot.motor_lift.setTargetPosition((int) lift_start_encoder_value - 10100);
        robot.motor_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor_lift.setPower(1);
        while (robot.motor_lift.isBusy() && navi.drive && opModeIsActive()) {
            navi.step();
            output();
        }

        //Drive to high junction
        navi.drive_to_pos(0,82.0*fz,0.1,0.3);
        while (navi.drive && opModeIsActive()) {
            navi.step();
            output();
        }

        //wait again and let cone down
        robot.servo3.setPosition(0.2);
        //wait 0.5 seconds
        startTime = (new Date()).getTime();
        while (startTime+500 > (new Date()).getTime() && opModeIsActive()) {
        }


        //open claw & let cones
        robot.servo1.setPosition(0.0);
        robot.servo2.setPosition(0.4);


        //wait 0.5seconds
        startTime = (new Date()).getTime();
        while (startTime+500 > (new Date()).getTime() && opModeIsActive()) {
        }


        //Drive back & arm down & servos back
        navi.drive_to_pos(0.0, 110*fz,0.2,0.3);
        robot.servo1.setPosition(0.4);
        robot.servo2.setPosition(0.0);
        robot.servo3.setPosition(0.1);
        robot.motor_lift.setTargetPosition((int) lift_start_encoder_value - 1000);
        robot.motor_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor_lift.setPower(1);
        while (robot.motor_lift.isBusy() && navi.drive && opModeIsActive()) {
            navi.step();
            output();
        }


        //wait 1 second
        startTime = (new Date()).getTime();
        while (startTime+1000 > (new Date()).getTime() && opModeIsActive()) {
        }

        //Lift Claw
        robot.servo3.setPosition(0.3);

        //wait 1 second
        startTime = (new Date()).getTime();
        while (startTime+1000 > (new Date()).getTime() && opModeIsActive()) {
        }

    }

    public void driveToTerminal(){
        int fx = Quadrant()%2==0?1:-1;
        int fz = Quadrant()<2?-1:1;

        //Drive back, against wall
        navi.drive_to_pos(0.0,187.0*fz,0.3,0.2);
        while (navi.drive && opModeIsActive()) {
            navi.step();
            output();
        }

        // reset z
        navi.position_z = 160*fz;
        navi.position_x = 0;

        // drive 5 cm forward
        navi.drive_to_pos(0,152*fz,0.3,0.3);
        while (navi.drive && opModeIsActive()) {
            navi.step();
            output();
        }

        // drive to terminal
        navi.drive_to_pos(180*fx,152*fz,0.3,0.3);
        while (navi.drive && opModeIsActive()) {
            navi.step();
            output();
        }
        //drive back
        navi.drive_to_pos(180*fx,170*fz,0.3,0.3);
        while (navi.drive && opModeIsActive()) {
            navi.step();
            output();
        }

        //arm down
        robot.servo3.setPosition(0.1);
        robot.motor_lift.setTargetPosition((int) lift_start_encoder_value);
        robot.motor_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor_lift.setPower(1);
        while (robot.motor_lift.isBusy() && opModeIsActive()) {
        }

        robot.servo4.setPosition(0.0);

        //wait 1 second
        long startTime = (new Date()).getTime();
        while (startTime+1000 > (new Date()).getTime() && opModeIsActive()) {
        }


    }

    public void driveToZone() {
        int fx = Quadrant()%2==0?1:-1;
        int fz = Quadrant()<2?-1:1;

        double dx = 30;
        double dz = 80;

        if (signal_detected == 0) {
            driveToTerminal();
            return;
        }

        //decide what Quadrant for side driving
        if (signal_detected == 0){
            driveToTerminal();
            return;
        }
        if (Quadrant() % 2 == 1) {
            dx *= -1;
        }
        if (Quadrant() > 1) {
            dz *= -1;
        }
        // drive to side
        navi.drive_to_pos(navi.position_x + dx, navi.position_z, 0.3, 0.1);
        while (navi.drive && opModeIsActive()) {
            navi.step();
            output();
        }

        // rotate
        navi.drive_setMotors(0,0,(false?-1:1),0.5);
        // Quadrant() < 2 ? -90 : 90
        while(!(
                navi.rotation_y < (Quadrant() < 2 ? -85 : 95)  &&
                navi.rotation_y > (Quadrant() < 2 ? -95 : 85)) && opModeIsActive()
        ) {
            navi.stepRotation();
            navi.stepPos();
        }

        // drive forwards
        navi.drive_to_pos(navi.position_x*fx, (navi.position_z + dz)*fz, 0.4, 0.1);
        while (navi.drive && opModeIsActive()) {
            navi.step();
            output();
        }

        // get value & zone
        switch (signal_detected) {
            case 1:
                dx = 0;
                break;
            case 2:
                dx = 60;
                break;
            case 3:
                dx = 120;
                break;
            default:
                dx = 0;
        }
        // drive to specific zone
        if (Quadrant() == 1) {
            if (dx != 60)
                dx -= 120;
            else
                dx = -60;
        } else if (Quadrant() == 2) {
            if (dx != 60) {
                if (dx == 0)
                    dx = 120;
                else
                    dx = 0;
            }
        } else if (Quadrant() == 3) {
            dx *= -1;
        }
        //drive to specific zone
        navi.drive_to_pos((navi.position_x + dx)*fx, navi.position_z*fz, 0.3, 0.1);
        while (navi.drive && opModeIsActive()) {
            navi.step();
        }
    }

    protected void initTFod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, SIGNAL_LABELS);
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.0,16.0/9.0);
        } else {
            telemetry.addLine("ERROR");
            telemetry.update();
        }
    }

    public void detectSignal() {
        int fz = Quadrant() < 2 ? -1:1;

        long startTime = (new Date()).getTime();
        String max_signal = "";
        float max_confidence = 0.0f;
        signal_detected = 0;
        while (max_confidence < 0.5 && opModeIsActive() && startTime+4000 > (new Date()).getTime()) {
            List<Recognition> updateRecognitions = tfod.getUpdatedRecognitions();
            if (updateRecognitions != null) {
                for (Recognition recognition : updateRecognitions) {
                    telemetry.addData("%s (%f)", recognition.getLabel(), recognition.getConfidence());
                    if (max_signal.equals(recognition.getLabel()))
                        max_confidence += 0.10f;
                    else if (recognition.getConfidence() > max_confidence) {
                        max_signal = recognition.getLabel();
                        max_confidence = recognition.getConfidence();
                    }
                }

            }
            telemetry.update();
        }
        for (int i=0; i<SIGNAL_LABELS.length; i++) {
            if (max_signal.equals(SIGNAL_LABELS[i])) {
                signal_detected = i;
                break;
            }
        }
    }

    //public abstract BaseHardwareMap initializeHardwareMap();
    
    public abstract void run();
}
