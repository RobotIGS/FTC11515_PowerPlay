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

    public void detect() {
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
            int s2 = Math.max(Math.max(red_det, green_det), Math.max(blue_det, not_det));
            if(s2 >= 3){
                if (s2 == red_det) {

                    //drive to parking zone two
                } else if (s2 == green_det) {
                    //drive to parking zone three
                } else if (s2 == blue_det) {
                    //drive to parking zone one
                }
                else {
                    //drive in terminal
                }



            }



        }
    }
}



