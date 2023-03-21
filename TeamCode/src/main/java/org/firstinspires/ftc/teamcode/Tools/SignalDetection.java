package org.firstinspires.ftc.teamcode.Tools;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Date;

public class SignalDetection {
    private OpenCVPipeline pipeline;

    public SignalDetection(OpenCvCamera phoneCam){
        pipeline = new OpenCVPipeline();
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

    public int detect(double min_d) {

        double blue  = pipeline.meancolor.val[2];
        double red   = pipeline.meancolor.val[0];
        double green = pipeline.meancolor.val[1];
        double color_sum = 0;
        int color_max = 0;

        if (red > green){
            color_max = 2;
        } else{
            color_max = 3;
        }
        if ((color_max == 2? red : green) < blue){
            color_max = 1;
        }
        // m - d = (a + b) / 2

        color_sum += (color_max == 1? -blue : blue/2);
        color_sum += (color_max == 2? -red : red/2);
        color_sum += (color_max == 3? -green : green/2);
        color_sum += min_d;

        if (color_sum > 0)
            color_max = 0;

        return color_max;
    }
}



