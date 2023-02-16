package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareMaps.WebcamHardwareMap;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Date;

@TeleOp
public class TestOpenCV extends BaseTeleOp {
    WebcamHardwareMap webcamHardwareMap;
    OpenCvCamera phoneCam;
    PowerPlayPipeline pipeline;

    long startTime;

    @Override
    public void initialize() {
        webcamHardwareMap = new WebcamHardwareMap(hardwareMap);
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(webcamHardwareMap.webcam);
        pipeline = new PowerPlayPipeline();
        phoneCam.setPipeline(pipeline);
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    @Override
    public void loop() {
        telemetry.addData("red  ",(int)pipeline.meancolor.val[0]);
        telemetry.addData("green",(int)pipeline.meancolor.val[1]);
        telemetry.addData("blue ",(int)pipeline.meancolor.val[2]);
        telemetry.update();
        startTime = (new Date()).getTime();
        while (startTime+1000 > (new Date()).getTime()) {
        }
    }
}

class PowerPlayPipeline extends OpenCvPipeline {
    Mat output;
    int rect_sizex = 30;
    int rect_sizey = 40;
    int rect_dx = 0;
    int rect_dy = 40;
    Scalar rect_color = new Scalar(255,0,0);
    Point p1 = new Point();
    Point p2 = new Point();

    Scalar meancolor;

    @Override
    public Mat processFrame(Mat input) {
        output = input.clone();
        p1 = new Point(input.width()/2 - rect_sizex/2 +rect_dx,input.height()/2 - rect_sizey/2 +rect_dy);
        p2 = new Point(input.width()/2 + rect_sizex/2 +rect_dx,input.height()/2 + rect_sizey/2 +rect_dy);

        Imgproc.rectangle(output, p1, p2, rect_color,4);
        meancolor = Core.mean(input.submat(new Rect(p1,p2)));
        return output;
    }
}