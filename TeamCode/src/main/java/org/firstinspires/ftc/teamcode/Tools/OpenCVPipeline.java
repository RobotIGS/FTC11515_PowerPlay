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
        private Rect cropRect;
        Scalar rect_color = new Scalar(255, 0, 0);
        Point p1 = new Point();
        Point p2 = new Point();
        //TODO get/set/constructor
        Scalar meancolor;

        public OpenCVPipeline() {
            updateRect();
        }

        public void updateRect() {
            p1 = new Point(- rect_sizex / 2 + rect_dx, rect_sizey / 2 + rect_dy);
            p2 = new Point(rect_sizex / 2 + rect_dx, rect_sizey / 2 + rect_dy);
            cropRect = new Rect(p1,p2);
        }

        @Override
        public Mat processFrame(Mat input) {
            output = input.clone();
            p1.x += input.size(0)/2;
            p1.y += input.size(1)/2;
            p2.x += input.size(0)/2;
            p2.y += input.size(1)/2;

            Imgproc.rectangle(output, p1, p2, rect_color, 4);
            meancolor = Core.mean(input.submat(cropRect));
            return output;
        }

    }
