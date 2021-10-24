package org.firstinspires.ftc.teamcode.CustomCV;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class detector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();

    public enum Location {
        LEFT,
        RIGHT,
        MID
    }
    private Location location;

    static final Rect LEFT_ROI = new Rect(new Point(0, 35), new Point(70, 100));

    static final Rect RIGHT_ROI = new Rect(new Point(80, 35), new Point(150, 100));

    static final Rect MID_ROI = new Rect(new Point(160, 35), new Point(230, 100));

    static double thresh = 0.4;

    public detector(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Scalar lowHSV = new Scalar(15, 50, 70);
        Scalar highHSV = new Scalar(25, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat right = mat.submat(RIGHT_ROI);
        Mat mid = mat.submat(MID_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;
        double midValue = Core.sumElems(mid).val[0] / MID_ROI.area() / 255;

        left.release();
        right.release();
        mid.release();

        boolean objectLeft = leftValue > thresh;
        boolean objectRight = rightValue > thresh;
        boolean objectMid = midValue > thresh;

        if (objectLeft) {
            location = Location.LEFT;
            telemetry.addData("pos", "left");
        }else if (objectRight){
            location = Location.RIGHT;
            telemetry.addData("pos", "right");
        }else if(objectMid){
            location = Location.MID;
            telemetry.addData("pos", "middle");
        }

        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");
        telemetry.addData("Middle percentage", Math.round(midValue * 100) + "%");

        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorObject = new Scalar(255, 0, 0);
        Scalar colorNoObject = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT? colorNoObject:colorObject);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? colorNoObject:colorObject);
        Imgproc.rectangle(mat, MID_ROI, location == Location.MID? colorNoObject:colorObject);

        return mat;
    }

    public Location getLocation() {
        return location;
    }
}
