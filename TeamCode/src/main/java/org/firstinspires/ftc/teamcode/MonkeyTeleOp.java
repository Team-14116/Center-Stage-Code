package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Monkey Mania", group="Learning")
public class MonkeyTeleOp extends OpMode{

    /* Declare OpMode members. */
    BaseRobot robot   = new BaseRobot(); // use the class created to define a Robot's hardware
    // could also use HardwarePushbotMatrix class.

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Robot Ready");    //

        // Set to Run without Encoder for Tele Operated
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.pullUp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      //  robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.grip.setPosition(0.4);
        robot.pivot.setPosition(0.625);
        robot.launch.setPosition(1);



    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        int armPos = robot.arm.getCurrentPosition();

        double y = -gamepad1.left_stick_y * 0.6; // Remember, this is reversed!
        double x = gamepad1.left_stick_x * 0;
        double rx = gamepad1.right_stick_x * 0.6;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        robot.leftFront.setPower(frontLeftPower);
        robot.rightFront.setPower(frontRightPower);
        robot.leftRear.setPower(backLeftPower);
        robot.rightRear.setPower(backRightPower);

        if(gamepad1.dpad_right) {
            robot.leftFront.setPower(0.5);
            robot.leftRear.setPower(-0.5);
            robot.rightFront.setPower(-0.5);
            robot.rightRear.setPower(0.5);
        }

        if(gamepad1.dpad_left) {
            robot.leftFront.setPower(-0.5);
            robot.leftRear.setPower(0.5);
            robot.rightFront.setPower(0.5);
            robot.rightRear.setPower(-0.5);
        }

        // main arm
        if(gamepad1.y && armPos < -1500) {
            robot.arm.setPower(1);
            robot.pivot.setPosition(0.9);
        } else if(gamepad1.y && armPos > -1500) {
            robot.arm.setPower(0.3);
            robot.pivot.setPosition(0.9);
        } else if (gamepad1.a && armPos < -1500) {
            robot.arm.setPower(-1);
            robot.pivot.setPosition(0.625);
        } else if(gamepad1.a && armPos > -1500) {
            robot.arm.setPower(-0.3);
            robot.pivot.setPosition(0.625);
        } else {
            robot.arm.setPower(0.01);
        }

        // claw mechanism
        if(gamepad1.left_bumper){
            robot.grip.setPosition(0.4);
        } else if(gamepad1.right_bumper) {
            robot.grip.setPosition(1);
        }

        if(gamepad1.b) {
            robot.pivot.setPosition(0.625);
        } else if(gamepad1.x) {
            robot.pivot.setPosition(0.9);
        }

        if(gamepad1.dpad_up) {
            robot.launch.setPosition(1);
        } if(gamepad1.dpad_down) {
            robot.launch.setPosition(0);
        }




        // pull up arm
        robot.pullUp.setPower(gamepad1.right_trigger);
        robot.pullUp.setPower(-gamepad1.left_trigger);
        telemetry.addData("arm pos", robot.arm.getCurrentPosition());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}