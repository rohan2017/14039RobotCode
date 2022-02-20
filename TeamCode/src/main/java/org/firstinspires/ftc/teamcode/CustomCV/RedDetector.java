package org.firstinspires.ftc.teamcode.CustomCV;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class RedDetector extends OpenCvPipeline {

    private Telemetry telemetry;
    private Mat workingMat = new Mat();
    private Mat frameSized = new Mat();

    public enum Location {
        LEFT,
        RIGHT,
        MID
    }

    private Location location;

    private double[] leftValue, rightValue, midValue;
    private double leftDist, rightDist, midDist;

    private static final Rect LEFT_ROI = new Rect(new Point(150, 40), new Point(200, 90)); // 90, 60, 150, 100
    private static final Rect MID_ROI = new Rect(new Point(240, 90), new Point(280, 140)); // 180, 60, 250, 125
    private static final Rect RIGHT_ROI = new Rect(new Point(310, 105), new Point(319, 170)); // 260, 105, 319, 170

    private static final double[] targetColor = {125, 153, 145}; // in HSV

    @Override
    public Mat processFrame(Mat input) {
        // Downsize for faster processing & averaged pixels
        //Imgproc.resize(input, frameSized, new Size(0, 0), 0.5, 0.5, Imgproc.INTER_AREA);
        Imgproc.cvtColor(input, workingMat, Imgproc.COLOR_RGB2HSV);

        // Select ROIs out of image
        Mat left = workingMat.submat(LEFT_ROI);
        Mat right = workingMat.submat(RIGHT_ROI);
        Mat mid = workingMat.submat(MID_ROI);

        // Average pixel values within ROIs
        leftValue = Core.sumElems(left).val.clone();
        rightValue = Core.sumElems(right).val.clone();
        midValue = Core.sumElems(mid).val.clone();

        left.release();
        right.release();
        mid.release();

        averageValue(leftValue, LEFT_ROI.area());
        averageValue(rightValue, RIGHT_ROI.area());
        averageValue(midValue, MID_ROI.area());

        // Transform to find color distance
        leftDist = colorDistance(leftValue);
        rightDist = colorDistance(rightValue);
        midDist = colorDistance(midValue);

        // Find closest pixel
        if(leftDist < rightDist) { // cant be right
            if(leftDist < midDist) { // cant be mid
                location = Location.LEFT;
            }else { // cant be left
                location = Location.MID;
            }
        }else { // cant be left
            if(rightDist < midDist) { // cant be mid
                location = Location.RIGHT;
            }else { // cant be right
                location = Location.MID;
            }
        }

        Scalar colorObject = new Scalar(255, 0, 0);
        Scalar colorNoObject = new Scalar(0, 255, 0);

        Imgproc.rectangle(input, LEFT_ROI, location == Location.LEFT? colorNoObject:colorObject);
        Imgproc.rectangle(input, RIGHT_ROI, location == Location.RIGHT? colorNoObject:colorObject);
        Imgproc.rectangle(input, MID_ROI, location == Location.MID? colorNoObject:colorObject);

        return input;
    }

    public static double min(double a, double b, double c) {
        return Math.min(Math.min(a, b), c);
    }
    public Location getLocation() {
        return location;
    }


    private void averageValue(double[] pixel, double area) {
        for(int i=0; i<3; i++) {
            pixel[i] = pixel[i] / area;
        }
    }

    private double colorDistance(double[] pixel) {
        double distance = 0;
        distance += Math.pow(pixel[0] - targetColor[0], 2)*0.7;
        distance += Math.pow(pixel[1] - targetColor[1], 2)*0.2;
        distance += Math.pow(pixel[2] - targetColor[2], 2)*0.1;
        return Math.sqrt(distance);
    }

    public double[] getRightValue(){
        return rightValue;
    }
    public double[] getLeftValue(){
        return leftValue;
    }
    public double[] getMidValue(){
        return midValue;
    }

}
