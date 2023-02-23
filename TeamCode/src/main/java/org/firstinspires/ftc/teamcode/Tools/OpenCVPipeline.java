package org.firstinspires.ftc.teamcode.Tools;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class OpenCVPipeline extends OpenCvPipeline {
        Mat output;
        int rect_sizex = 30;
        int rect_sizey = 40;
        int rect_dx = 0;
        int rect_dy = 40;
        Scalar rect_color = new Scalar(255, 0, 0);
        Point p1 = new Point();
        Point p2 = new Point();
        //TODO get/set/constructor
        Scalar meancolor;

        @Override
        public Mat processFrame(Mat input) {
            output = input.clone();
            p1 = new Point(input.width() / 2 - rect_sizex / 2 + rect_dx, input.height() / 2 - rect_sizey / 2 + rect_dy);
            p2 = new Point(input.width() / 2 + rect_sizex / 2 + rect_dx, input.height() / 2 + rect_sizey / 2 + rect_dy);

            Imgproc.rectangle(output, p1, p2, rect_color, 4);
            meancolor = Core.mean(input.submat(new Rect(p1, p2)));
            return output;
        }

    }
