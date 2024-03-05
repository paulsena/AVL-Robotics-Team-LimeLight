package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

/**
 * Use EasyOpenCV Simulator to help test
 *
 * Detects mascot using HSV color scale.
 * Filters out all non blue or red values and makes a black and white mask.
 * Averages white values in a bounding box and determines which spike mark the mascot is on.
 */
public class DetectMascotVisionPipeline extends OpenCvPipeline {

    // 0 Red, 1 Blue
    public int teamColor = 0;

    // HSV color ranges to filter for. Makes a mask
    // Red has two ranges, so handle special
    public Scalar redLower = new Scalar(170, 70, 50); // HSV threshold bounds
    public Scalar redUpper = new Scalar(180, 255, 255);

    public Scalar redLower2 = new Scalar(0, 150, 100); // HSV threshold bounds
    public Scalar redUpper2 = new Scalar(10, 255, 255);

    public Scalar blueLower = new Scalar(100, 180, 130); // HSV threshold bounds
    public Scalar blueUpper = new Scalar(135, 255, 255);

    // Rectangle regions to be scanned
    // Rect(top left x, top left y, bottom right x, bottom right y)
    // Camera Dimensions: 1920 x 1080
    public static Rect leftSpikeBoundingBox = new Rect(80, 200, 125, 125);
    public static Rect rightSpikeBoundingBox = new Rect(520, 200, 125, 125);

    public Scalar leftSpikeAverage;
    public Scalar rightSpikeAverage;
    public int mascotSpikePosition;


    private final Mat hsvMat = new Mat(); // converted image
    private final Mat binaryMat = new Mat(); // image analyzed after thresholding
    private final Mat binaryMat2 = new Mat(); // image analyzed after thresholding
    private Mat leftSpikeMat;
    private Mat rightSpikeMat;
    private final Mat finalMask = new Mat();

    @Override
    public Mat processFrame(Mat input) {

        // Convert from BGR to HSV
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        if (teamColor == 0) {
            // Red
            Core.inRange(hsvMat, redLower, redUpper, binaryMat);
            Core.inRange(hsvMat, redLower2, redUpper2, binaryMat2);
        } else {
            // Blue
            Core.inRange(hsvMat, blueLower, blueUpper, binaryMat);
        }

        // Combine masks. Needed for red b/c there are two color range masks

        Core.bitwise_or(binaryMat, binaryMat2, finalMask);

        //Defining areas to scan. These are new Mats
        leftSpikeMat = finalMask.submat(leftSpikeBoundingBox);
        rightSpikeMat = finalMask.submat(rightSpikeBoundingBox);

        // Calc average for each spike line area
        leftSpikeAverage = Core.mean(leftSpikeMat);
        rightSpikeAverage = Core.mean(rightSpikeMat);

        if (leftSpikeAverage.val[0] > 30) {
            mascotSpikePosition = 1;
        } else if (rightSpikeAverage.val[0] > 30) {
            mascotSpikePosition = 3;
        } else {
            mascotSpikePosition = 2;
        }


        // Draw bounding box for debugging
//        Imgproc.rectangle(finalMask, leftSpikeBoundingBox, new Scalar(200), 2);
//        Imgproc.rectangle(finalMask, rightSpikeBoundingBox, new Scalar(200), 2);

        return finalMask;
    }

    public int getMascotSpikePosition() {
        return mascotSpikePosition;
    }

    public void setTeamColor(int teamColor) {
        this.teamColor = teamColor;
    }

    public int getTeamColor() {
        return teamColor;
    }
}