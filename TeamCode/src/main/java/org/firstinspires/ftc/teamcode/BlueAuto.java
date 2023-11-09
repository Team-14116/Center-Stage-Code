package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="Blue Auto")
public class BlueAuto extends LinearOpMode {

    public ColorSensor leftSensor = null;
    public ColorSensor rightSensor = null;

    public DcMotor arm = null;
    public Servo   grip = null;
    public Servo   pivot = null;
    
    @Override
    public void runOpMode() throws InterruptedException {

        arm    = hardwareMap.dcMotor.get("arm");
        grip    = hardwareMap.servo.get("grip");
        pivot    = hardwareMap.servo.get("pivot");

        leftSensor = hardwareMap.colorSensor.get("leftColor");
        rightSensor = hardwareMap.colorSensor.get("rightColor");


        Pose2d startPose = new Pose2d(0,0,0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        Action move1 = drive.actionBuilder(startPose)
                .lineToX(10)
                .build();

        telemetry.addData("Status", "Ready to run");
        telemetry.addData("red left", leftSensor.red());
        telemetry.addData("green left", leftSensor.green());
        telemetry.addData("blue left", leftSensor.blue());

        telemetry.addData("red right", rightSensor.red());
        telemetry.addData("green right", rightSensor.green());
        telemetry.addData("blue right", rightSensor.blue());


        telemetry.update();

        waitForStart();







    }

}
