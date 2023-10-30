package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import android.app.Activity;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
@Autonomous(name="Autonomous", group="Learning")
//@Disabled

public class Autonomous3 extends LinearOpMode{


    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    // For Encoder Functions
    private double     COUNTS_PER_MOTOR_REV          = 753.2;    // eg: TETRIX Motor Encoder
    private final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    private double     WHEEL_DIAMETER_INCHES         = 4.0 ;     // For figuring circumference
    private double     COUNTS_PER_INCH               = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private double COUNTS_PER_DEGREE                 = COUNTS_PER_MOTOR_REV / 360;
    private ElapsedTime period = new ElapsedTime();
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftRear;
    public DcMotor rightRear;
    public DcMotor pullUp;
    public DcMotor arm;
    public Servo   grip;
    public Servo   pivot;
    public Servo   auto;
    public Servo   launch;

    int newLeftTarget;
    int newRightTarget;
    double newInches;

    //imu
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, correction, angle, gain = .075, turnCorrection, deltaAngle;


    @Override
    public void runOpMode() throws InterruptedException {

        leftFront   = hardwareMap.dcMotor.get("leftFront");
        rightFront  = hardwareMap.dcMotor.get("rightFront");
        leftRear     = hardwareMap.dcMotor.get("leftRear");
        rightRear    = hardwareMap.dcMotor.get("rightRear");
        pullUp    = hardwareMap.dcMotor.get("pullUp");
        arm    = hardwareMap.dcMotor.get("arm");

        grip    = hardwareMap.servo.get("grip");
        pivot    = hardwareMap.servo.get("pivot");
        auto    = hardwareMap.servo.get("auto");
        launch    = hardwareMap.servo.get("launch");


        leftFront.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightFront.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftRear.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightRear.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        pullUp.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        arm.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors



        // Set all motors to zero power
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        pullUp.setPower(0);
        arm.setPower(0);

        grip.setPosition(1);
        auto.setPosition(0);
        pivot.setPosition(0);
        launch.setPosition(1);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pullUp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.

        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        // wait for the start button to be pressed.

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());//
        telemetry.update();



        waitForStart();

        delay(0.1);

        drive(15, 0.5);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }

    // Sample Delay Code
    public void delay(double t) { // Imitates the Arduino delay function
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < t)) {
            // telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            // telemetry.update();
        }
    }
    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    public double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.


        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(double degrees, double power)
    {
        double  leftPower, rightPower;
        // restart imu movement tracking.

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees > getAngle())
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees < getAngle())
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;


        // set power to rotate.


        // rotate until turn is completed.
        if (degrees < getAngle())
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
                leftFront.setPower(leftPower);
                leftRear.setPower(leftPower);
                rightFront.setPower(rightPower);
                rightRear.setPower(rightPower);
                telemetry.addData("angle", getAngle());
                telemetry.update();
            }

            while (opModeIsActive() && getAngle() > degrees) {
                leftFront.setPower(leftPower);
                leftRear.setPower(leftPower);
                rightFront.setPower(rightPower);
                rightRear.setPower(rightPower);
                telemetry.addData("angle", getAngle());
                telemetry.update();
            }
        }
        else  {  // left turn.
            while (getAngle() < degrees) {
                leftFront.setPower(leftPower);
                leftRear.setPower(leftPower);
                rightFront.setPower(rightPower);
                rightRear.setPower(rightPower);
                telemetry.addData("angle", getAngle());
                telemetry.update();
            }
        }


        // turn the motors off.
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);

        // wait for rotation to stop.
        // reset angle tracking on new heading.


    }


    public void driveStraightInches(double speed,
                                    double inches,
                                    double timeoutS) {

        double leftCorrection = 0;
        double rightCorrection = 0;

        //reverse inches

        leftRear.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);

        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set to Limit of DRIVE_SPEED
       /* if (Math.abs(speed) > DRIVE_SPEED) {
            speed = DRIVE_SPEED; //
        }
        */
        // Ensure that the opmode is still active
        if (opModeIsActive()) { //if this function was on the autonomous code
            //if (true) {       // Swapped out to include in WalkerBaseRobot

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftRear.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightTarget = rightRear.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

            leftRear.setTargetPosition(newLeftTarget);
            rightRear.setTargetPosition(newRightTarget);
            leftFront.setTargetPosition(newLeftTarget);
            rightFront.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            period.reset();

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (period.seconds() < timeoutS) && (leftRear.isBusy() && rightRear.isBusy() && leftFront.isBusy() && rightFront.isBusy())){


                checkDirection();
                rightRear.getCurrentPosition();

                if(inches > 0) {
                    leftCorrection = Math.abs(speed) - correction;
                    rightCorrection = Math.abs(speed) + correction;
                } else if (inches < 0) {
                    leftCorrection = Math.abs(speed) + correction;
                    rightCorrection = Math.abs(speed) - correction;
                }

                //telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.addData("4 leftCorrection", leftCorrection);
                telemetry.addData("5 rightCorrection", rightCorrection);
                telemetry.addData("currentRightRearPosition", rightRear.getCurrentPosition());
                telemetry.addData("currentLeftRearPosition", leftRear.getCurrentPosition());
                telemetry.addData("currentRightFrontPosition", rightFront.getCurrentPosition());
                telemetry.addData("currentLeftFrontPosition", leftFront.getCurrentPosition());
                telemetry.update();

                leftRear.setPower(leftCorrection);
                rightRear.setPower(rightCorrection);
                leftFront.setPower(leftCorrection);
                rightFront.setPower(rightCorrection);
                // Wait for Sequence to complete
            }

            // Stop all motion;
            leftRear.setPower(0);
            rightRear.setPower(0);
            leftFront.setPower(0);
            rightFront.setPower(0);

            newInches = (-leftRear.getCurrentPosition() + newLeftTarget)/COUNTS_PER_INCH;

            // Turn off RUN_TO_POSITION
            leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //  sleep(250);   // optional pause after each move


        }
    }


    public void strafeInches(double speed,
                             double inches,
                             double timeoutS) {

        int newLeftTarget;
        int newRightTarget;
        double leftCorrection;
        double rightCorrection;

        // Reverse inches
        // inches = inches * -1;

        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Set to Limit of DRIVE_SPEED


        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftTarget = leftRear.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightTarget = rightRear.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            newRightTarget = leftFront.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            newLeftTarget = rightFront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            leftRear.setTargetPosition(newLeftTarget);
            rightRear.setTargetPosition(newRightTarget);
            leftFront.setTargetPosition(newRightTarget);
            rightFront.setTargetPosition(newLeftTarget);

            // Turn On RUN_TO_POSITION
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            period.reset();


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ((opModeIsActive()) && (period.seconds() < timeoutS) &&
                    (leftRear.isBusy())) {
                // Wait for Sequence to complete

                checkDirection();

                leftCorrection = Math.abs(speed) - correction;
                rightCorrection  = Math.abs(speed) + correction;

                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                //telemetry.addData("3 red", sensorColor.red());
                telemetry.addData("4 leftCorrection", leftCorrection);
                telemetry.addData("5 rightCorrection", rightCorrection);
                telemetry.update();

                leftRear.setPower(speed);
                rightRear.setPower(speed);
                leftFront.setPower(speed);
                rightFront.setPower(speed);
                // Wait for Sequence to complete

            }
            telemetry.addData("status", "stoppingmotors");
            telemetry.update();
            // Stop all motion;

            // Turn off RUN_TO_POSITION
            leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            leftRear.setPower(0);
            rightRear.setPower(0);
            leftFront.setPower(0);
            rightFront.setPower(0);
            //  sleep(250);   // optional pause after each move
        }


    }


    public void correctedTurn (double degrees, double power){

        resetAngle();
        rotate(degrees, power);

        turnCorrection = -getAngle() + degrees;
        resetAngle();
        rotate(turnCorrection, power/2);

        turnCorrection = -getAngle() + turnCorrection;
        resetAngle();
        rotate(turnCorrection, power/4);

        turnCorrection = -getAngle() + turnCorrection;
        resetAngle();
        rotate(turnCorrection, power/6);



        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        // reset angle tracking on new heading.
        resetAngle();

    }

    public void armEncoder(double speed,
                           double inches,
                           double timeoutS) {
        int newArmTarget;

        arm.setPower(0);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Ensure that the opmode is still active
        if (opModeIsActive()) { //if this function was on the autonomous code
            //if (true) {       // Swapped out to include in WalkerBaseRobot

            // Determine new target position, and pass to motor controller
            newArmTarget = arm.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            arm.setTargetPosition(newArmTarget);

            // Turn On RUN_TO_POSITION
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            period.reset();

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (arm.isBusy()) ){
                arm.setPower(speed);
            }

            // Stop all motion;


            // Turn off RUN_TO_POSITION
            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //  sleep(250);   // optional pause after each move

            arm.setPower(0.05);
        }
    }

    public void drive (double inches, double power) {
        driveStraightInches(power, inches, 5);
       // driveStraightInches(power/2, newInches, 3);
        //driveStraightInches(power/2, newInches, 3);
    }


}
