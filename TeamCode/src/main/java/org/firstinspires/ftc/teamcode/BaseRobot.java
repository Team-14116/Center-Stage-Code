package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class BaseRobot {
    /* Public Motors and Servos */
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftRear = null;
    public DcMotor rightRear = null;
    public DcMotor pullUp = null;
    public DcMotor arm = null;
    public Servo   grip = null;
    public Servo   pivot = null;
    public Servo   auto = null;
    public Servo   launch = null;


    // For Encoder Functions
    private double           COUNTS_PER_MOTOR_REV          = 1440 ;    // eg: TETRIX Motor Encoder
    private final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private double           WHEEL_DIAMETER_INCHES         = 4.0 ;     // For figuring circumference
    private double           COUNTS_PER_INCH               = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private double COUNTS_PER_DEGREE                 = COUNTS_PER_MOTOR_REV / 360;

    // Local OpMode members
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    // Constructor - leave this blank for now
    public BaseRobot () {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors.  Assign Names that match the setup on the RC Phone
        leftFront   = hwMap.dcMotor.get("leftFront");
        rightFront  = hwMap.dcMotor.get("rightFront");
        leftRear     = hwMap.dcMotor.get("leftRear");
        rightRear    = hwMap.dcMotor.get("rightRear");
        pullUp    = hwMap.dcMotor.get("pullUp");
        arm    = hwMap.dcMotor.get("arm");

        grip    = hwMap.servo.get("grip");
        pivot    = hwMap.servo.get("pivot");
        auto    = hwMap.servo.get("auto");
        launch    = hwMap.servo.get("launch");


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


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pullUp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }



}