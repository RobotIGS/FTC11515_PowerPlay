package org.firstinspires.ftc.teamcode.Tools;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class JunctionOpenCVPipeline extends OpenCvPipeline {
    Mat output;
    int rect_sizex = 74;
    int rect_sizey = 50;
    int rect_side_sizex = 8;
    int rect_dx = 0; // 20
    int rect_dy = 0; // -25;
    private Rect cropRectR;
    private Rect cropRectM;
    private Rect cropRectL;
    Scalar rect_colorL = new Scalar(255, 0, 0);
    Scalar rect_colorM = new Scalar(0, 255, 0);
    Scalar rect_colorR = new Scalar(0, 0, 255);
    Point p1 = new Point();
    Point p2 = new Point();
    Point p3 = new Point();
    Point p4 = new Point();
    //TODO get/set/constructor
    public Scalar[] returnScalars = new Scalar[3];

    public void updateRect() {

    }

    @Override
    public Mat processFrame(Mat input) {
        output = input.clone();
        p1 = new Point(-rect_sizex/2 - rect_side_sizex -rect_dx + input.size(1)/2, -rect_sizey/2-rect_dy + input.size(0)/2);
        p2 = new Point(-rect_sizex/2 -rect_dx + input.size(1)/2, rect_sizey/2-rect_dy + input.size(0)/2);
        p3 = new Point(rect_sizex/2 -rect_dx + input.size(1)/2, -rect_sizey/2-rect_dy + input.size(0)/2);
        p4 = new Point(rect_sizex/2 + rect_side_sizex-rect_dx + input.size(1)/2, rect_sizey/2-rect_dy + input.size(0)/2);
        cropRectL = new Rect(p1,p2);
        cropRectM = new Rect(p2,p3);
        cropRectR = new Rect(p3,p4);

        returnScalars[0] = Core.mean(input.submat(cropRectL));
        returnScalars[1] = Core.mean(input.submat(cropRectM));
        returnScalars[2] = Core.mean(input.submat(cropRectR));


        if (returnScalars[0].val[0] > 200) {
            Imgproc.rectangle(output, p1, p2, rect_colorL, -1);
        } else {
            Imgproc.rectangle(output, p1, p2, rect_colorL, 2);
        }
        if (returnScalars[1].val[0] > 200) {
            Imgproc.rectangle(output, p2, p3, rect_colorM, -1);
        } else {
            Imgproc.rectangle(output, p2, p3, rect_colorM, 2);
        }
        if (returnScalars[2].val[0] > 200) {
            Imgproc.rectangle(output, p3, p4, rect_colorR, -1);
        } else {
            Imgproc.rectangle(output, p3, p4, rect_colorR, 2);
        }

        return output;
    }

}