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

@Autonomous(name="Red Auto Left")
public class RedAuto2 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime period = new ElapsedTime();

    public ColorSensor leftSensor = null;
    public ColorSensor rightSensor = null;

    public DcMotor arm = null;
    public Servo gripRight = null;
    public Servo gripLeft = null;
    public Servo   pivotRight = null;
    public Servo   pivotLeft = null;
    MecanumDrive drive = null;

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startPose = new Pose2d(0,0,0);

        drive = new MecanumDrive(hardwareMap, startPose);

        arm    = hardwareMap.dcMotor.get("arm");
        arm.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        gripRight = hardwareMap.servo.get("gripRight");
        gripLeft = hardwareMap.servo.get("gripLeft");
        pivotLeft    = hardwareMap.servo.get("pivotLeft");
        pivotRight    = hardwareMap.servo.get("pivotRight");

        leftSensor = hardwareMap.colorSensor.get("leftColor");
        rightSensor = hardwareMap.colorSensor.get("rightColor");

        pivotLeft.setPosition(0.3); //up
        pivotRight.setPosition(0.7); // up
        gripLeft.setPosition(0); // close
        gripRight.setPosition(1); //close


        Action move1 = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(20,5))
                .lineToX(30)
                .waitSeconds(0.5)
                .build();

        Action move1a = drive.actionBuilder(new Pose2d(30, 5, 0))
                .strafeTo(new Vector2d(23, 14))
                .build();


        Action move1b = drive.actionBuilder(new Pose2d(23, 14, 0))
                .strafeTo(new Vector2d(15, 15))
                .build();

        Action move1c = drive.actionBuilder(new Pose2d(15, 15, 0))
                .strafeTo(new Vector2d(25, 35))
                .turn(Math.toRadians(90))
                .build();

        Action move1d = drive.actionBuilder(new Pose2d(25, 35, Math.toRadians(90)))
                .lineToY(50)
                .build();

        Action move1e = drive.actionBuilder(new Pose2d(25, 50, Math.toRadians(90)))
                .lineToY(35)
                .build();

        Action move2 = drive.actionBuilder(new Pose2d(30, 5, 0))
                .strafeTo(new Vector2d(25, 0))
                .build();

        Action move22 = drive.actionBuilder(new Pose2d(25, 0, 0))
                .lineToX(34)

                .build();

        Action move23 = drive.actionBuilder(new Pose2d(34, 0, 0))
                .lineToX(29)

                .build();

        Action move2a = drive.actionBuilder(new Pose2d(29, 0, 0))
                .strafeTo(new Vector2d(21, 2))

                .build();

        Action move2b = drive.actionBuilder(new Pose2d(21, 2, 0))
                .strafeTo(new Vector2d(30, 35))
                .turn(Math.toRadians(90))
                .build();

        Action move2c = drive.actionBuilder(new Pose2d(30, 35, Math.toRadians(90)))
                .lineToY(50)
                .build();

        Action move2d = drive.actionBuilder(new Pose2d(30, 50, Math.toRadians(90)))
                .lineToY(35)
                .build();


        Action move3 = drive.actionBuilder(new Pose2d(29, 0, 0))
                .turn(Math.toRadians(-90))
                .lineToY(-16)
                .lineToY(-5)
                .build();

        Action move3a = drive.actionBuilder(new Pose2d(30, -5, Math.toRadians(-90)))
                .lineToY(10)

                .build();

        Action move3b = drive.actionBuilder(new Pose2d(30, 10, Math.toRadians(-90)))
                .strafeTo(new Vector2d(39, 25))
                .turn(Math.toRadians(180))
                .build();

        Action move3c = drive.actionBuilder(new Pose2d(39, 25, Math.toRadians(90)))
                .lineToY(42)
                .build();

        Action move3d = drive.actionBuilder(new Pose2d(39, 42, Math.toRadians(90)))
                .lineToY(27)
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
        delay(0.1);

        Actions.runBlocking(move1);


        if(leftSensor.red() > leftSensor.green() && leftSensor.red() > leftSensor.blue()) {
            delay(0.25);
            Actions.runBlocking(move1a);
            pivotLeft.setPosition(.6); // down
            pivotRight.setPosition(.4); // down
            delay(0.25);
            gripLeft.setPosition(1);// open right grip
            delay(0.25);
            pivotLeft.setPosition(0.3); //up
            pivotRight.setPosition(0.7); // up
            Actions.runBlocking(move1b);
            /*
            delay(0.5);
            Actions.runBlocking(move1c);

            arm.setPower(1);
            delay(0.25);
            arm.setPower(0.1);


            Actions.runBlocking(move1d);

            delay(0.5);
            grip2.setPosition(0.45);
            delay(0.5);

            Actions.runBlocking(move1e);
            */
        } else {
            Actions.runBlocking(move2);
            Actions.runBlocking(move22);
            telemetry.addData("red", leftSensor.red());
            telemetry.update();
            if(rightSensor.red() > rightSensor.blue() && rightSensor.red() > leftSensor.green()) {
                delay(0.25);
                Actions.runBlocking(move23);
                pivotLeft.setPosition(.6); // down
                pivotRight.setPosition(.4); // down
                delay(0.25);
                gripLeft.setPosition(1);// open right grip
                delay(0.25);
                pivotLeft.setPosition(0.3); //up
                pivotRight.setPosition(0.7); // up
                Actions.runBlocking(move2a);
                delay(0.5);
                /*
                Actions.runBlocking(move2b);

                arm.setPower(1);
                delay(0.25);
                arm.setPower(0.1);


                Actions.runBlocking(move2c);
                delay(0.5);
                grip2.setPosition(0.45);

                delay(0.5);
                Actions.runBlocking(move2d);

                */
            } else {

                Actions.runBlocking(move3);
                delay(0.25);
                pivotLeft.setPosition(.6); // down
                pivotRight.setPosition(.4); // down
                delay(0.25);
                gripLeft.setPosition(1);// open right grip
                delay(0.25);
                pivotLeft.setPosition(0.3); //up
                pivotRight.setPosition(0.7); // up
                Actions.runBlocking(move3a);

                /*
                delay(0.5);
                Actions.runBlocking(move3b);
                delay(0.5);
                arm.setPower(1);
                delay(0.25);
                arm.setPower(0.1);


                Actions.runBlocking(move3c);
                delay(0.5);
                grip2.setPosition(0.45);

                delay(0.25);

                Actions.runBlocking(move3d);

                */
            }
        }






    }

    public void delay(double t) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < t)) {

        }
    }

}
