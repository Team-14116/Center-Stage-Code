package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="Red Auto")
public class RedAuto extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime period = new ElapsedTime();

    public ColorSensor leftSensor = null;
    public ColorSensor rightSensor = null;

    public DcMotor arm = null;
    public Servo   grip = null;
    public Servo   pivot = null;
    MecanumDrive drive = null;
    
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startPose = new Pose2d(0,0,0);

        drive = new MecanumDrive(hardwareMap, startPose);

        arm    = hardwareMap.dcMotor.get("arm");
        arm.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        grip    = hardwareMap.servo.get("grip");
        pivot    = hardwareMap.servo.get("pivot");

        leftSensor = hardwareMap.colorSensor.get("leftColor");
        rightSensor = hardwareMap.colorSensor.get("rightColor");

        grip.setPosition(1);
        pivot.setPosition(0.625);


        Action move1 = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(20,-5))
                .lineToX(30)
                .waitSeconds(0.5)
                .build();

            Action move1a = drive.actionBuilder(new Pose2d(30, -5, 0))
                    .strafeTo(new Vector2d(23, -14))
                    .build();


            Action move1b = drive.actionBuilder(new Pose2d(23, -14, 0))
                    .strafeTo(new Vector2d(15, -15))
                    .strafeTo(new Vector2d(27, -18))
                    .build();

            Action move1c = drive.actionBuilder(new Pose2d(27, -18, 0))
                    .lineToX(15)
                    .strafeTo(new Vector2d(20, -40))
                    .turn(Math.toRadians(-90))
                    .build();

            Action move1d = drive.actionBuilder(new Pose2d(20, -40, Math.toRadians(-90)))
                    .lineToY(50)
                    .build();

            Action move1e = drive.actionBuilder(new Pose2d(20, -50, Math.toRadians(-90)))
                    .strafeTo(new Vector2d(5, -49))
                    .build();

        Action move2 = drive.actionBuilder(new Pose2d(30, -5, 0))
                .strafeTo(new Vector2d(25, -1))
                .build();

        Action move22 = drive.actionBuilder(new Pose2d(25, -1, 0))
                .lineToX(33)
                .build();

            Action move2a = drive.actionBuilder(new Pose2d(33, -1, 0))
                    .strafeTo(new Vector2d(25, -2))
                    .strafeTo(new Vector2d(36, -7))
                    .build();

            Action move2b = drive.actionBuilder(new Pose2d(36, -7, 0))
                    .lineToX(20)
                    .strafeTo(new Vector2d(30, -40))
                    .turn(Math.toRadians(-90))
                    .build();

            Action move2c = drive.actionBuilder(new Pose2d(30, -40, Math.toRadians(-90)))
                    .lineToY(-50)
                    .build();

            Action move2d = drive.actionBuilder(new Pose2d(25, -50, Math.toRadians(-90)))
                    .strafeTo(new Vector2d(5, -49))
                    .build();


        Action move3 = drive.actionBuilder(new Pose2d(33, -1, 0))
                .lineToX(28)
                .turn(Math.toRadians(90))
                .lineToY(13)
                .lineToY(8)
                .build();

            Action move3a = drive.actionBuilder(new Pose2d(28, 8, Math.toRadians(90)))
                    .strafeTo(new Vector2d(29, 2))
                    .strafeTo(new Vector2d(32, 11))
                    .build();

            Action move3b = drive.actionBuilder(new Pose2d(32, 11, Math.toRadians(90)))
                    .lineToY(-10)
                .strafeTo(new Vector2d(32, -30))
                    .turn(Math.toRadians(180))
                .build();

            Action move3c = drive.actionBuilder(new Pose2d(32, -30, Math.toRadians(-90)))
                .lineToY(-50)
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

        Actions.runBlocking(move1);

        if(rightSensor.red() > rightSensor.green() && rightSensor.red() > rightSensor.blue()) {
            Actions.runBlocking(move1a);
            grip.setPosition((0.2));
            Actions.runBlocking(move1b);
            delay(0.5);
            grip.setPosition(1);
            delay(0.5);
            Actions.runBlocking(move1c);

            arm.setPower(1);
            delay(0.5);
            arm.setPower(0.1);
            pivot.setPosition(0.9);

            Actions.runBlocking(move1d);

            delay(0.5);
            grip.setPosition(0.2);
            delay(0.5);

            Actions.runBlocking(move1e);

        } else {
            Actions.runBlocking(move2);
            Actions.runBlocking(move22);
            if(leftSensor.red() > leftSensor.green() && leftSensor.red() > leftSensor.blue()) {
                grip.setPosition(0);
                Actions.runBlocking(move2a);
                delay(0.5);
                grip.setPosition(1);
                delay(0.5);
                Actions.runBlocking(move2b);

                arm.setPower(1);
                delay(0.5);
                arm.setPower(0.1);
                pivot.setPosition(0.9);

                Actions.runBlocking(move2c);
                delay(0.5);
                grip.setPosition(0.2);

                delay(0.5);
                Actions.runBlocking(move2d);


            } else {

                Actions.runBlocking(move3);
                delay(0.25);
                grip.setPosition(0.2);
                delay(0.25);
                Actions.runBlocking(move3a);
                delay(0.25);
                grip.setPosition(1);

                delay(0.5);
                Actions.runBlocking(move3b);

                arm.setPower(1);
                delay(0.5);
                arm.setPower(0.1);
                pivot.setPosition(0.9);

                Actions.runBlocking(move3c);
                delay(0.5);
                grip.setPosition(0.2);

                delay(0.25);



            }
        }






    }

    public void delay(double t) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < t)) {

        }
    }

}
