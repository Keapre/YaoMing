package org.firstinspires.ftc.teamcode.OpMode.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

/**
 * Bench test / calibration for the three goBILDA laser present-detectors, used digitally.
 *
 * <p>Reads the three lasers straight off their {@link DigitalChannel} ports (no Robot/Sensors init,
 * so it runs on the bench with nothing else configured). For each laser it shows the raw digital
 * line, the interpreted "ball present" state, and how long it has held that state — everything you
 * need to set each sensor's distance threshold and confirm wiring/polarity.
 *
 * <h3>How to calibrate</h3>
 * <ol>
 *   <li>Run this, put a ball at the real staging distance in front of one laser.</li>
 *   <li>Adjust that laser's threshold (knob/button on the sensor) until "present" flips to YES with
 *       the ball there and back to no when it's removed.</li>
 *   <li>Watch "ms in state" — it should be steady, not flickering, at the switching distance. If it
 *       chatters, back the threshold off slightly.</li>
 * </ol>
 *
 * <p>Convention matches {@code Sensors}: <b>ball present = HIGH</b>. If a sensor turns out inverted,
 * flip {@link #PRESENT_IS_HIGH} to re-check without rewiring (then fix it in {@code Sensors} to match).
 */
@Config
@TeleOp(name = "Laser Sensor Test", group = "tuning")
public class LaserSensorTest extends LinearOpMode {

    public static String LEFT_NAME = "laserLeft";
    public static String RIGHT_NAME = "laserRight";
    public static String TRANSFER_NAME = "laserTransfer";

    /** Ball present == digital HIGH (true). Flip to test/verify inverted wiring. */
    public static boolean PRESENT_IS_HIGH = true;

    private DigitalChannel left, right, transfer;

    // per-sensor state tracking
    private final boolean[] present = new boolean[3];
    private final long[] changedAt = new long[3];
    private final int[] edges = new int[3];

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        left = tryGet(LEFT_NAME);
        right = tryGet(RIGHT_NAME);
        transfer = tryGet(TRANSFER_NAME);

        long now = System.currentTimeMillis();
        for (int i = 0; i < 3; i++) changedAt[i] = now;

        telemetry.addLine("Laser Sensor Test ready.");
        telemetry.addLine("Put a ball in front of each laser; watch 'present'.");
        telemetry.addData("left", left == null ? "NOT FOUND" : "ok");
        telemetry.addData("right", right == null ? "NOT FOUND" : "ok");
        telemetry.addData("transfer", transfer == null ? "NOT FOUND" : "ok");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            reportSensor(0, "LEFT (pos1)", left);
            reportSensor(1, "RIGHT (pos2)", right);
            reportSensor(2, "TRANSFER (pos3)", transfer);

            boolean all = present[0] && present[1] && present[2];
            telemetry.addLine();
            telemetry.addData("ALL present", all ? "YES (3 balls)" : "no");
            telemetry.addData("PRESENT_IS_HIGH", PRESENT_IS_HIGH);
            telemetry.update();
        }
    }

    private void reportSensor(int idx, String label, DigitalChannel ch) {
        if (ch == null) {
            telemetry.addData(label, "NOT FOUND (check config name)");
            return;
        }
        boolean raw = ch.getState();
        boolean isPresent = PRESENT_IS_HIGH == raw; // present when raw == HIGH (or LOW if flag off)

        if (isPresent != present[idx]) {
            present[idx] = isPresent;
            changedAt[idx] = System.currentTimeMillis();
            edges[idx]++;
        }
        long held = System.currentTimeMillis() - changedAt[idx];

        // dashboard-graphable numeric (1 present / 0 clear) plus a readable line
        telemetry.addData(label + " present#", isPresent ? 1 : 0);
        telemetry.addData(label, String.format("%s  | raw=%s | %d ms in state | edges=%d",
                isPresent ? "BALL PRESENT" : "clear", raw ? "HIGH" : "LOW", held, edges[idx]));
    }

    private DigitalChannel tryGet(String name) {
        try {
            DigitalChannel ch = hardwareMap.get(DigitalChannel.class, name);
            ch.setMode(DigitalChannel.Mode.INPUT);
            return ch;
        } catch (Exception e) {
            return null;
        }
    }
}
