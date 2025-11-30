package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous
public class AprilTagWebcamExample extends LinearOpMode {

    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();


    @Override
    public void runOpMode() {

        // Initialize
        aprilTagWebcam.init(hardwareMap, telemetry);

        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        // Main loop
        while (opModeIsActive()) {

            // Update camera detections
            aprilTagWebcam.update();

            // Get ALL detected tags
            List<AprilTagDetection> detections = aprilTagWebcam.getDetectedTags();

            // Get pattern from the list
            String pattern = aprilTagWebcam.getPattern(detections);

            // Telemetry display
            telemetry.addLine("=== Detections ===");
            if (detections.isEmpty()) {
                telemetry.addLine("No Tags Detected");
            } else {
                for (AprilTagDetection det : detections) {
                    telemetry.addData("Tag ID", det.id);
                    aprilTagWebcam.displayDetectionTelemetry(det);
                }
            }

            telemetry.addLine();
            telemetry.addData("Pattern", pattern.equals("") ? "None" : pattern);

            telemetry.update();
        }
    }
}
