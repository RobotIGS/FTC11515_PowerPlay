package org.firstinspires.ftc.teamcode.Tools;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Date;

enum JUNCTION_DRIVE {
    JUNCTION_DRIVE_NONE
};

public class JunctionDrive {
    public JunctionOpenCVPipeline pipeline;

    public JunctionDrive(OpenCvCamera phoneCam){
        pipeline = new JunctionOpenCVPipeline();
        phoneCam.setPipeline(pipeline);
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

    }

    public int step() {
        return 0;
    }
}



