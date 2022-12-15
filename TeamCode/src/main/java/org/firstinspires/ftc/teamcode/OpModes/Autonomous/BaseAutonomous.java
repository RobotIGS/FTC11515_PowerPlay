package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.HardwareMaps.BaseHardwareMap;
import org.firstinspires.ftc.teamcode.HardwareMaps.WebcamHardwareMap;

import java.util.Date;
import java.util.List;
import java.util.stream.IntStream;

public abstract class BaseAutonomous extends LinearOpMode {
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

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        run();
    }

    protected void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
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
