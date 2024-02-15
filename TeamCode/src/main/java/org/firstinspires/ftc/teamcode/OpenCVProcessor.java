package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.time.Year;

public class OpenCVProcessor {
    private final OpenCvCamera webcam;

    public OpenCVProcessor(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(new Pipeline());
    }

    public void startStreaming() {
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            public void onError(int errorCode) {
                Log.e("OpenCVProcessor", "Error opening camera: " + errorCode);
            }
        });
    }

    public void stopStreaming() {
        webcam.stopStreaming();
    }

    public static class Pipeline extends OpenCvPipeline {
        Mat output = new Mat();
        private final Rect roi1 = new Rect(50, 100, 100, 100); // Define your ROI coordinates here
        private final Rect roi2 = new Rect(150, 100, 100, 100); // Define your ROI coordinates here


        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGBA2BGR);

            Mat roi1Mat = new Mat(input, roi1);
            Mat roi2Mat = new Mat(input, roi2);

            int bluePixelsRoi1 = countBluePixels(roi1Mat);
            int bluePixelsRoi2 = countBluePixels(roi2Mat);

            int redPixelsRoi1 = countRedPixels(roi1Mat);
            int redPixelsRoi2 = countRedPixels(roi2Mat);

            // Draw rectangles over the ROIs
            Scalar rectangleColor = new Scalar(0, 255, 0); // Green color
            int rectangleThickness = 2;

            input.copyTo(output);
            Imgproc.cvtColor(output, output, Imgproc.COLOR_RGBA2RGB);

            Imgproc.rectangle(output, roi1.tl(), roi1.br(), rectangleColor, rectangleThickness);
            Imgproc.rectangle(output, roi2.tl(), roi2.br(), rectangleColor, rectangleThickness);

            // Draw text above the rectangles
            Scalar textColor = new Scalar(255, 255, 255); // White color
            double textSize = 0.4;
            int textThickness = 1;
            int textLineType = Imgproc.LINE_AA; // Anti-aliased line
            Point textPositionRoi1 = new Point(roi1.x, roi1.y - 10); // Position above the rectangle
            Point textPositionRoi2 = new Point(roi2.x, roi2.y - 10); // Position above the rectangle

            String textRoi1 = "Blue: " + bluePixelsRoi1 + ", Red: " + redPixelsRoi1;
            String textRoi2 = "Blue: " + bluePixelsRoi2 + ", Red: " + redPixelsRoi2;

            Imgproc.putText(output, textRoi1, textPositionRoi1, Imgproc.FONT_HERSHEY_SIMPLEX, textSize, textColor, textThickness, textLineType, false);
            Imgproc.putText(output, textRoi2, textPositionRoi2, Imgproc.FONT_HERSHEY_SIMPLEX, textSize, textColor, textThickness, textLineType, false);

            // Do something with the counts
            return output;
        }

        private int countBluePixels(Mat img) {
            // Convert the image from BGR to HSV
            Mat hsv = new Mat();
            Imgproc.cvtColor(img, hsv, Imgproc.COLOR_BGR2HSV);

            // Define the range for blue color in HSV
            Scalar lowerBlue = new Scalar(100, 150, 0);
            Scalar upperBlue = new Scalar(140, 255, 255);

            // Threshold the HSV image to get only blue colors
            Mat blueMask = new Mat();
            Core.inRange(hsv, lowerBlue, upperBlue, blueMask);

            // Count the blue pixels
            return Core.countNonZero(blueMask);
        }

        private int countRedPixels(Mat img) {
            // Convert the image from BGR to HSV
            Mat hsv = new Mat();
            Imgproc.cvtColor(img, hsv, Imgproc.COLOR_BGR2HSV);

            // Define the range for red color in HSV
            Scalar lowerRed = new Scalar(0, 70, 50);
            Scalar upperRed = new Scalar(10, 255, 255);

            // Threshold the HSV image to get only red colors
            Mat redMask = new Mat();
            Core.inRange(hsv, lowerRed, upperRed, redMask);

            // Count the red pixels
            return Core.countNonZero(redMask);
        }
    }
}