package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.HardwareMaps.BaseHardwareMap;
import org.firstinspires.ftc.teamcode.HardwareMaps.FullHardwareMap;
import org.firstinspires.ftc.teamcode.HardwareMaps.GyroHardwareMap;
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

    private void initialize() {
        robot = new FullHardwareMap(hardwareMap);
        lift_start_encoder_value  = robot.motor_lift.getCurrentPosition();
        gyro = new GyroHardwareMap(hardwareMap);
        navi = new FieldNavigation(robot, gyro,
                Quadrant() % 2 == 0 ? 88 : -88,
                Quadrant() / 2 >= 1 ? 165 : -165,
                Quadrant() < 2 ? -90 : 90, // changed
                0.8/180., 1e-7

        );
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

    public void detectSignal() {
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
        navi.drive_rel(-150,0,0.2,2);
        while (navi.drive && opModeIsActive()) {
            navi.step();


        }
    }

    public void driveToJunctionHigh() {
        initialize();

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

        //Drive 10cm forward
        navi.drive_to_pos(88.0, -160.0, 0.3, 0.3);
        while (navi.drive && opModeIsActive()) {
            navi.step();
            output();
        }

        //Drive to x = 0
        navi.drive_to_pos(0.0,-160.0,0.3,0.3);
        while (navi.drive && opModeIsActive()) {
            navi.step();
            output();
        }

        //Drive next to high junction
        navi.drive_to_pos(0.0,-85.0,0.3,0.3);
        while (navi.drive && opModeIsActive()) {
            navi.step();
            output();
        }

        //Lift motor arm to junction
        robot.motor_lift.setTargetPosition((int) lift_start_encoder_value - 10100);
        robot.motor_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor_lift.setPower(1);
        while (robot.motor_lift.isBusy() && opModeIsActive()) {
        }

        //Drive to high junction
        navi.drive_to_pos(0.0,-82.0,0.3,0.3);
        while (navi.drive && opModeIsActive()) {
            navi.step();
            output();
        }

        //wait again
        startTime = (new Date()).getTime();
        while (startTime+1000 > (new Date()).getTime() && opModeIsActive()) {
        }

        //open claw & let cones
        robot.servo1.setPosition(0.0);
        robot.servo2.setPosition(0.4);


        //wait 1,5 seconds
        startTime = (new Date()).getTime();
        while (startTime+1500 > (new Date()).getTime() && opModeIsActive()) {
        }

        //Drive back
        navi.drive_to_pos(0.0, -84,0.3,0.3);
        while (navi.drive && opModeIsActive()) {
            navi.step();
            output();
        }

        //motor arm in original pos
        robot.servo1.setPosition(0.4);
        robot.servo2.setPosition(0.0);
        robot.servo3.setPosition(0.1);
        robot.motor_lift.setTargetPosition((int) lift_start_encoder_value - 5);
        robot.motor_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor_lift.setPower(1);
        while (robot.motor_lift.isBusy() && opModeIsActive()) {
        }

        //wait 1 second
        startTime = (new Date()).getTime();
        while (startTime+1000 > (new Date()).getTime() && opModeIsActive()) {
        }

    }

    public void driveToZone() {
        navi.drive_to_pos(-29.0,90.0,0.2,0.3);
        while (navi.drive && opModeIsActive()) {
            navi.step();
        }

        navi.drive_to_pos(-88.0,90.0,0.2,0.3);
        while (navi.drive && opModeIsActive()) {
            navi.step();
        }

        navi.drive_to_pos(-149.0,90.0,0.2,0.3);
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

    protected abstract void initialize();

    public void detectSignal() {
        long startTime = (new Date()).getTime();
        String max_signal;
        float max_confidence = 0.0f;
        while (opModeIsActive()) {// && startTime+5000 > (new Date()).getTime()) {
            max_signal = "";
            List<Recognition> updateRecognitions = tfod.getUpdatedRecognitions();
            if (updateRecognitions != null) {
                telemetry.addData("n", updateRecognitions.size());
                telemetry.update();
                for (Recognition recognition : updateRecognitions) {
                    if (max_signal.equals(recognition.getLabel()))
                        max_confidence = 0.95f;
                    else if (recognition.getConfidence() > max_confidence) {
                        max_signal = recognition.getLabel();
                        max_confidence = recognition.getConfidence();
                    }
                }
                for (int i=0; i<SIGNAL_LABELS.length; i++) {
                    if (max_signal.equals(SIGNAL_LABELS[i])) {
                        signal_detected = i;
                        break;
                    }
                }
            }
            if (signal_detected != 0)
                break;
        }
    }

    //public abstract BaseHardwareMap initializeHardwareMap();
    
    public abstract void run();
}
