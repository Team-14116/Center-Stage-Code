package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous(name="AutoTestZoning", group="Pushbot")
public class AutoTestZoning extends LinearOpMode {
    OpenCVProcessor openCVProcessor;

    @Override
    public void runOpMode() {
        openCVProcessor = new OpenCVProcessor(hardwareMap);
        openCVProcessor.startStreaming(); // Start streaming


        waitForStart();

        while (opModeIsActive()) {
            // Autonomous code here
        }

        openCVProcessor.stopStreaming(); // Stop streaming
    }
}