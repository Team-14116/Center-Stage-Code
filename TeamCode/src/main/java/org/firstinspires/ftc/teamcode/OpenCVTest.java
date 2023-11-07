package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class OpenCVTest extends OpMode{

    OpenCvWebcam webcam1 = null;
    @Override
    public void init() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = (hardwareMap.get(WebcamName.class, "Webcam1"));
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam1.setPipeline(new examplePipeline());

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(640, 360, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    @Override
    public void loop() {

    }

    class examplePipeline extends OpenCvPipeline {
        Mat YCbCr = new Mat();
        Mat leftCrop = new Mat();
        Mat midCrop = new Mat();
        Mat rightCrop = new Mat();
        double leftavgfin;
        double midavgfin;
        double rightavgfin;

        Mat outPut = new Mat();
        Scalar leftColor = new Scalar(255.0, 0.0, 0.0);
        Scalar midColor = new Scalar(0.0, 255.0, 0.0);
        Scalar rightColor = new Scalar(0.0, 0.0, 255.0);

        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("pipeline running");

            Rect leftRect = new Rect(1, 1, 119, 639);
            Rect midRect = new Rect(120, 1, 119, 639);
            Rect rightRect = new Rect(240, 1, 119, 639);

            input.copyTo(outPut);
            Imgproc.rectangle(outPut, leftRect, leftColor, 2);
            Imgproc.rectangle(outPut, midRect, midColor, 2);
            Imgproc.rectangle(outPut, rightRect, rightColor, 2);

            leftCrop = YCbCr.submat(leftRect);
            midCrop = YCbCr.submat(midRect);
            rightCrop = YCbCr.submat(rightRect);

            Core.extractChannel(YCbCr, leftCrop, 2);
            Core.extractChannel(YCbCr, midCrop, 2);
            Core.extractChannel(YCbCr, rightCrop, 2);

            Scalar leftAvg = Core.mean(leftCrop);
            Scalar midAvg = Core.mean(midCrop);
            Scalar rightAvg = Core.mean(rightCrop);

            leftavgfin = leftAvg.val[0];
            midavgfin = midAvg.val[0];
            rightavgfin = rightAvg.val[0];

            if((leftavgfin > midavgfin) & (leftavgfin > rightavgfin)) {
                telemetry.addLine("left");
            } else if((midavgfin > leftavgfin) && (midavgfin > rightavgfin)){
                telemetry.addLine("middle");
            } else if((rightavgfin > leftavgfin) && (rightavgfin > midavgfin)) {
                telemetry.addLine("right");
            }

            return(outPut);
        }
    }

}


