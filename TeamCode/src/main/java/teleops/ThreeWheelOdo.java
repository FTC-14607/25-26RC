package teleops;

import org.firstinspires.ftc.teamcode.util.hardware.Encoder;

public final class ThreeWheelOdo {

    private final Encoder left, right, perp;
    private final double trackWidth, centerOffset, inchesPerTick;

    private double lastL, lastR, lastP;

    public double x = 0.0;       // inches
    public double y = 0.0;       // inches
    public double heading = 0.0; // radians

    public ThreeWheelOdo(Encoder left, Encoder right, Encoder perp,
                         double trackWidth, double centerOffset,
                         int ticksPerRev, double wheelDiameterIn) {
        this.left = left;
        this.right = right;
        this.perp = perp;
        this.trackWidth = trackWidth;
        this.centerOffset = centerOffset;
        this.inchesPerTick = Math.PI * wheelDiameterIn / ticksPerRev;

        this.lastL = left.getCurrentPosition();
        this.lastR = right.getCurrentPosition();
        this.lastP = perp.getCurrentPosition();
    }

    public void update() {
        double l = left.getCurrentPosition();
        double r = right.getCurrentPosition();
        double p = perp.getCurrentPosition();

        double dL = (l - lastL) * inchesPerTick;
        double dR = (r - lastR) * inchesPerTick;
        double dP = (p - lastP) * inchesPerTick;

        lastL = l; lastR = r; lastP = p;

        double dTheta  = (dR - dL) / trackWidth;
        double dS      = (dL + dR) / 2.0;
        double dStrafe = dP - centerOffset * dTheta;

        double thetaMid = heading + dTheta / 2.0;
        double dx = dS * Math.cos(thetaMid) - dStrafe * Math.sin(thetaMid);
        double dy = dS * Math.sin(thetaMid) + dStrafe * Math.cos(thetaMid);

        heading += dTheta;
        x += dx;
        y += dy;
    }
}