package teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.robots.Tyrone;

@TeleOp(name = "Odo Viewer", group = "Test")
public class OdoViewer extends LinearOpMode {

    private Tyrone bot;
    private FtcDashboard dash;

    @Override
    public void runOpMode() {
        bot = new Tyrone(this);

        // Mirror phone tel
        // emetry to Dashboard
        dash = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        telemetry.addLine("Odometry ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            bot.update(); // updates Tyrone.currentPose internally

            Pose2d p = Tyrone.currentPose;
            if (p != null) {
                double x = p.getX();
                double y = p.getY();
                double headingDeg = p.getHeading();

                telemetry.addData("x (in)", "%.2f", x);
                telemetry.addData("y (in)", "%.2f", y);
                telemetry.addData("θ (deg)", "%.1f", headingDeg);

                TelemetryPacket pkt = new TelemetryPacket();
                Canvas c = pkt.fieldOverlay();

                c.strokeCircle(x, y, 7);
                double hx = x + 10 * Math.cos(Math.toRadians(headingDeg));
                double hy = y + 10 * Math.sin(Math.toRadians(headingDeg));
                c.strokeLine(x, y, hx, hy);
                dash.sendTelemetryPacket(pkt);
            } else {
                telemetry.addLine("Pose not yet available");
            }

            telemetry.update();
        }
    }
}