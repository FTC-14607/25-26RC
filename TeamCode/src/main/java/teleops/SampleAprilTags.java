package teleops;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.vision.TeamVisionUtilities;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

@TeleOp(name = "AprilTag_Test", group = "Test")
public class SampleAprilTags extends LinearOpMode {
    // CV
    private static final int RESOLUTION_WIDTH  = 640; // TODO: fiddle resolution
    private static final int RESOLUTION_HEIGHT = 480;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private AprilTagDetection aprilTagDetections = null; // holds data for detected AprilTag

    // Fill these with your field’s IDs once verified in telemetry.
    // Example:
    //  put(11, "GPP");
    //  put(12, "PGG");
    //  put(13, "PPG");
    private static final Map<Integer, String> MOTIF_BY_ID = new HashMap<Integer, String>() {{
        // put(11, "GPP");
        // put(12, "PGG");
        // put(13, "PPG");
        // ...add the rest for your venue/field
    }};

    @Override
    public void runOpMode() {
        // Build the AprilTag processor via your team utility (should already use current season library).
        aprilTagProcessor = TeamVisionUtilities.getAprilTagProcessor();

        initVisionPortal();
        telemetry.addLine("VisionPortal Initialized.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> detectedTags = aprilTagProcessor.getDetections();

            // Select the best single tag to represent the OBELISK front face:
            // prioritize smallest absolute bearing; break ties with smallest absolute yaw.
            AprilTagDetection front = chooseFrontFacingTag(detectedTags);

            // Usual per-tag telemetry (optional — still shows everything you see)
            for (AprilTagDetection detectedTag : detectedTags) {
                if (detectedTag.metadata != null && detectedTag.ftcPose != null) {
                    telemetry.addData("Target",  "ID %d (%s)", detectedTag.id, detectedTag.metadata.name);
                    telemetry.addData("Range",   "%5.1f inches",  detectedTag.ftcPose.range);
                    telemetry.addData("Pitch",   "%5.0f degrees", detectedTag.ftcPose.pitch);   // up/down
                    telemetry.addData("Yaw",     "%5.0f degrees", detectedTag.ftcPose.yaw);     // left/right
                    telemetry.addData("Bearing", "%5.0f degrees", detectedTag.ftcPose.bearing); // turn
                    telemetry.addLine("--------------------------------------");
                }
            }

            // Pattern/MOTIF resolution from the chosen tag (if any)
            if (front != null) {
                telemetry.addData("Chosen Tag", "ID %d (%s)", front.id,
                        (front.metadata != null ? front.metadata.name : "no metadata"));
                String motif = MOTIF_BY_ID.get(front.id);
                if (motif != null) {
                    telemetry.addData("MOTIF (pattern)", motif);
                } else {
                    telemetry.addData("MOTIF (pattern)", "Unknown → add ID %d to MOTIF_BY_ID", front.id);
                }
                telemetry.addLine("--------------------------------------");
                aprilTagDetections = front; // keep last chosen if you want to use elsewhere
            } else {
                telemetry.addLine("No clear front-facing tag — slide until only one is visible.");
            }

            telemetry.addData("FPS", "%4.1f", visionPortal.getFps()); // get FPS
            telemetry.update();
        }

        // End vision cleanly
        if (visionPortal != null) {
            // If you want to conserve resources mid-OpMode, you can stop/resume streaming, but here we just close.
            visionPortal.close();
        }
    }

    private void initVisionPortal() {
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .enableLiveView(false)
                .setCameraResolution(new Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT))
                .build();
    }

    /**
     * Choose the tag most likely to be the OBELISK front face:
     * - has metadata + pose
     * - minimal |bearing| (closest to straight ahead)
     * - tie-breaker: minimal |yaw| (most head-on)
     */
    private AprilTagDetection chooseFrontFacingTag(List<AprilTagDetection> detections) {
        AprilTagDetection best = null;
        double bestScore = Double.POSITIVE_INFINITY;

        if (detections == null) return null;

        for (AprilTagDetection d : detections) {
            if (d == null || d.metadata == null || d.ftcPose == null) continue;

            double score = Math.abs(d.ftcPose.bearing) + 0.2 * Math.abs(d.ftcPose.yaw);
            if (score < bestScore) {
                bestScore = score;
                best = d;
            }
        }
        return best;
    }
}