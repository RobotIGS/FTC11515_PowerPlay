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
    ROT_RIGHT,
    ROT_LEFT,
    DRIVE_FORW,
    SCORE,
    END2,
    SKIP
};

enum JUNCTION_DRIVE_STATE {
    START,
    ROT00,
    ROT01,
    ROT10,
    ROT11,
    FOR
}

public class JunctionDrive {
    public JunctionOpenCVPipeline pipeline;

    private JUNCTION_DRIVE_STATE q = JUNCTION_DRIVE_STATE.START;

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

    public boolean isStick(int pos) {
        return (pipeline.returnScalars[pos].val[0] > 200);
    }

    public JUNCTION_DRIVE step() {
        if (q == JUNCTION_DRIVE_STATE.START) {
            if (isStick(1))
                return JUNCTION_DRIVE.SCORE;
            else {
                if (isStick(0)) {
                    q = JUNCTION_DRIVE_STATE.ROT01;
                } else if (isStick(2)) {
                    q = JUNCTION_DRIVE_STATE.ROT11;
                } else {
                    return JUNCTION_DRIVE.SCORE;
                }
                return JUNCTION_DRIVE.SKIP;
            }
        }

        else if (q == JUNCTION_DRIVE_STATE.ROT00) {
            if (isStick(0) || isStick(2) || false) { // TODO: max rot
                q = JUNCTION_DRIVE_STATE.ROT01;
                return JUNCTION_DRIVE.SKIP;
            }
            return JUNCTION_DRIVE.ROT_LEFT;
        }

        else if (q == JUNCTION_DRIVE_STATE.ROT10) {
            if (isStick(0) || isStick(2) || false) { // TODO: max rot
                q = JUNCTION_DRIVE_STATE.ROT11;
                return JUNCTION_DRIVE.SKIP;
            }
            return JUNCTION_DRIVE.ROT_RIGHT;
        }

        else if (q == JUNCTION_DRIVE_STATE.ROT01) {
            if ((!isStick(0) && !isStick(2)) || false) { // TODO: max rot
                q = JUNCTION_DRIVE_STATE.FOR;
                return JUNCTION_DRIVE.SKIP;
            }
            return JUNCTION_DRIVE.ROT_RIGHT;
        }

        else if (q == JUNCTION_DRIVE_STATE.ROT11) {
            if ((!isStick(0) && !isStick(2)) || false) { // TODO: max rot
                q = JUNCTION_DRIVE_STATE.FOR;
                return JUNCTION_DRIVE.SKIP;
            }
            return JUNCTION_DRIVE.ROT_LEFT;
        }

        else if (q == JUNCTION_DRIVE_STATE.FOR) {
            if (false)                                           // TODO: max time
                return JUNCTION_DRIVE.END2;
            if (isStick(1)) {
                return JUNCTION_DRIVE.SCORE;
            }
            if (isStick(0)) {
                return JUNCTION_DRIVE.ROT_RIGHT;
            }
            if (isStick(2)) {
                return  JUNCTION_DRIVE.ROT_LEFT;
            }
            return JUNCTION_DRIVE.DRIVE_FORW;
        }

        return JUNCTION_DRIVE.END2;
    }
}



