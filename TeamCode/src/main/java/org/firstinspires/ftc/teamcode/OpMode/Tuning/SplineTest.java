package org.firstinspires.ftc.teamcode.OpMode.Tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.blob.Blob;
import com.blob.BlobParams;
import com.blob.geometry.PathGeometry;
import com.blob.geometry.Pose;
import com.blob.localization.SdkPinpointLocalizer;
import org.firstinspires.ftc.teamcode.Util.BlobConfig;

import org.firstinspires.ftc.teamcode.Util.Wrapper.GamePadController;

/**
 * Spline test for the published {@code com.blob} library.
 *
 * <p>Drives a <b>smooth S</b> as a single multi-waypoint path via {@link Blob#setPath(Pose...)} — one
 * centripetal Catmull-Rom curve that passes through every waypoint and is tangent-continuous across
 * them (no corner where the halves meet, unlike stitching two separate splines). The whole S is one
 * path with a single arc-length {@code progress}; {@code inPosition()} checks the final endpoint.
 *
 * <p>Visualization comes from the library itself via {@link Blob#drawCurrentPath} — it draws the
 * curve, its waypoints, the pure-pursuit carrot and the robot to the FTC Dashboard field overlay.
 *
 * <p>Geometry is relative to the START pose (offsets Dashboard-tunable), heading held constant.
 *
 * <ul>
 *   <li><b>A</b> — smooth S forward: start → w1 → w2 → w3 → w4</li>
 *   <li><b>B</b> — smooth S back: → w3 → w2 → w1 → start</li>
 *   <li><b>X</b> — stop / hold position</li>
 * </ul>
 */
@Config
@TeleOp(name = "Blob Spline Test", group = "test")
public class SplineTest extends LinearOpMode {

    // Waypoint offsets from the captured start pose (inches, odometry field frame). These lie ON the
    // S — the curve passes through each of them in order.
    public static double w1DX = 18,  w1DY = 20;
    public static double w2DX = 0,   w2DY = 40;
    public static double w3DX = -18, w3DY = 60;
    public static double w4DX = 0,   w4DY = 80;

    public static double lookahead = 6.0;   // pure-pursuit lookahead (inches)
    public static double maxPower = 1.0;    // drive power cap

    private enum Mode { IDLE, FWD, REV }

    @Override
    public void runOpMode() {
        GamePadController gg = new GamePadController(gamepad1);

        BlobParams params = BlobConfig.params();
        SdkPinpointLocalizer localizer = new SdkPinpointLocalizer(hardwareMap, params);
        Blob blob = new Blob(hardwareMap, params, localizer, Blob.State.PID);
        blob.drawCurrentPath = true;                          // library draws the path to Dashboard

        localizer.update();

        telemetry.addLine("Blob Spline Test — smooth S (one multi-waypoint path)");
        telemetry.addLine("A = S forward, B = S back, X = hold");
        telemetry.addLine("Place the robot with clear space, then press START.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        localizer.update();
        final double sx = localizer.getX();
        final double sy = localizer.getY();
        final double h = localizer.getRealHeading(); // command a constant heading

        Mode mode = Mode.IDLE;

        while (opModeIsActive()) {
            gg.update();
            params.splineLookahead = lookahead;
            blob.maxPower = maxPower;

            // Waypoints (rebuilt each loop so Dashboard edits take effect live).
            Pose start = new Pose(sx, sy, h);
            Pose w1 = new Pose(sx + w1DX, sy + w1DY, h);
            Pose w2 = new Pose(sx + w2DX, sy + w2DY, h);
            Pose w3 = new Pose(sx + w3DX, sy + w3DY, h);
            Pose w4 = new Pose(sx + w4DX, sy + w4DY, h);

            if (gg.aOnce()) mode = Mode.FWD;
            else if (gg.bOnce()) mode = Mode.REV;
            else if (gg.xOnce()) mode = Mode.IDLE;

            switch (mode) {
                case FWD:
                    blob.setPath(w1, w2, w3, w4);        // one smooth path through all four
                    if (blob.inPosition()) mode = Mode.IDLE;
                    break;
                case REV:
                    blob.setPath(w3, w2, w1, start);
                    if (blob.inPosition()) mode = Mode.IDLE;
                    break;
                case IDLE:
                default:
                    blob.setTargetPosition(new Pose(localizer.getX(), localizer.getY(), h));
                    break;
            }

            blob.update();

            PathGeometry path = blob.getPath();
            telemetry.addData("mode", mode);
            telemetry.addData("path active", blob.isPathActive());
            telemetry.addData("progress", "%.3f", blob.progress);
            telemetry.addData("pos", "%.1f, %.1f", localizer.getX(), localizer.getY());
            telemetry.addData("carrot", "%.1f, %.1f", blob.pidTargetX, blob.pidTargetY);
            telemetry.addData("end target", "%.1f, %.1f", blob.targetX, blob.targetY);
            telemetry.addData("in position", blob.inPosition());
            if (path != null) telemetry.addData("path length", "%.1f", path.getLength());
            telemetry.update();
        }
    }
}
