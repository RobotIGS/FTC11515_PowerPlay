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











        /*
        int red_det = 0;
        int green_det = 0;
        int blue_det = 0;
        int not_det = 0;
        for (int i = 0; i <= 5; i++) {
            int red = (int) pipeline.meancolor.val[0];
            int green = (int) pipeline.meancolor.val[1];
            int blue = (int) pipeline.meancolor.val[2];

            int s = Math.max(red, Math.max(green, blue));
            if (s == red) {
                red_det += 1;
            } else if (s == green) {
                green_det += 1;
            } else if (s == blue) {
                blue_det += 1;
            } else {
                not_det += 1;
            }
            long startTime = (new Date()).getTime();
            while (startTime + 200 > (new Date()).getTime()) {
            }
        }
        int s2 = Math.max(Math.max(red_det, green_det), Math.max(blue_det, not_det));
        if(s2 >= 3){
            if (s2 == green_det) {
                return 3;

            } else if (s2 == red_det) {
                return 2;

            } else if (s2 == blue_det) {
                return 1;
            }

        }
        return 0;
        
         */
    }
}



