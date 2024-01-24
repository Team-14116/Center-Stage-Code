package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import android.view.View;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagTest extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;

    BNO055IMU imu;
    AprilTagProcessor myAprilTagProcessor;
    VisionPortal myVisionPortal;


    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        myAprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            myVisionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), myAprilTagProcessor);
        } else {
            myVisionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, myAprilTagProcessor);
        }

    }   // end method initAprilTag()
    @Override
    public void runOpMode() {
      backLeftDrive = hardwareMap.dcMotor.get("leftRear");
      backRightDrive = hardwareMap.dcMotor.get("rightRear");
      frontLeftDrive = hardwareMap.dcMotor.get("leftFront");
      frontRightDrive = hardwareMap.dcMotor.get("rightFront");
      imu = hardwareMap.get(BNO055IMU.class, "imu");

      // Put initialization blocks here

      waitForStart();

      // Put run blocks here

      backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
      frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
      backLeftDrive.setPower(-1);
      backRightDrive.setPower(1);
      frontLeftDrive.setPower(-1);
      frontRightDrive.setPower(1);
      sleep(300);
      backLeftDrive.setPower(0);
      backRightDrive.setPower(0);
      frontLeftDrive.setPower(0);
      frontRightDrive.setPower(0);

      myAprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
      myVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "webcam"), myAprilTagProcessor); //check webcam name with team

      while (opModeIsActive()) {

        // Put loop blocks here

        List<AprilTagDetection> myAprilTagDetections = myAprilTagProcessor.getDetections();

        for (AprilTagDetection myAprilTagDetection : myAprilTagDetections) {
          telemetry.addLine("ID" + String.format("%d", myAprilTagDetection.id) + " " + (myAprilTagDetection.metadata.name));
          telemetry.addLine("X " + (myAprilTagDetection.ftcPose.x));
          telemetry.addLine("Y " + (myAprilTagDetection.ftcPose.y));
          telemetry.addLine("Z " + (myAprilTagDetection.ftcPose.z));
          telemetry.addLine("Range: " + (myAprilTagDetection.ftcPose.range));
          telemetry.addLine("Bearing " + (myAprilTagDetection.ftcPose.bearing));
          telemetry.addLine("Elevation " + (myAprilTagDetection.ftcPose.elevation));
          telemetry.addLine("Yaw" + (myAprilTagDetection.ftcPose.yaw));
          telemetry.addLine("Pitch " + (myAprilTagDetection.ftcPose.pitch));
          telemetry.addLine("Roll " + (myAprilTagDetection.ftcPose.roll));
        }
        telemetry.update();
      }
    }

}


