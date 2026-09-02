package org.firstinspires.ftc.teamcode.Hardware.Vision;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Hardware.Module;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Util.Globals.Phase;
import org.firstinspires.ftc.teamcode.Util.Info;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

@Config
public class Limelight implements Module {


    public static double STALE_TIMEOUT_MS = 250;
    public static double DETECT_CONF = 0.4;
    public static double HFOV_DEG = 54.5;
    public static int NUM_ZONES = 4;
    public static double ZONE_SWITCH_MIN_MARGIN = 1;
    public static int PREFERRED_ZONE = 0;
    public static int PREFERRED_ZONE_MARGIN = 2;
    public static double SMOOTHING_ALPHA = 0.15;
    public static int LOST_TARGET_FRAMES = 5;

    public static double TRACK_MATCH_DIST_DEG = 8.0;
    public static int MAX_MISSED_FRAMES = 5;
    public static double VELOCITY_ALPHA = 0.3;

    public static double SIDE_CENTER_HALF_DEG = 8.0;

    public static double LAMP_ON_POS = 1.0;
    public static double LAMP_OFF_POS = 0.0;

    private final Robot robot;
    private final Limelight3A ll;
    private final boolean enabled;
    private final ServoImplEx lamp;

    private double tx;
    private double ballCount;
    private double avgZoneSpeed;
    private int zone = -1;
    private int[] zoneCountsOut = new int[0];
    private int leftCount, centerCount, rightCount; // live view split
    private boolean hasTarget;

    private final List<Ball> tracks = new ArrayList<>();
    private double lastFrameTimeS = -1;
    private int lostCounter = 0;
    private Integer lockedZone = null;
    private int lockedZoneCount = 0;

    private final List<Trigger> triggers = new ArrayList<>();
    private boolean triggersEnabled = true;

    public Limelight(Robot robot) {
        this.robot = robot;

        Limelight3A tempLl = null;
        try {
            tempLl = robot.hw.get(Limelight3A.class, "limelight");
        } catch (Exception e) {
            Log.w("Limelight", "no 'limelight' in config; vision disabled: " + e.getMessage());
        }
        ll = tempLl;

        enabled = (ll != null) && (Info.phase == Phase.AUTONOMOUS);
        if (ll != null) {
            if (enabled) ll.start();
            else ll.stop();
        }

        ServoImplEx tempLamp = null;
        try {
            tempLamp = robot.hw.get(ServoImplEx.class, "lamp");
            tempLamp.setPwmRange(new PwmControl.PwmRange(500, 2500));
        } catch (Exception e) {
            tempLamp = null;
            Log.w("Limelight", "no 'lamp' in config: " + e.getMessage());
        }
        lamp = tempLamp;
        setLamp(false);
    }

    public void setLamp(boolean on) {
        if (lamp == null) return;
        lamp.setPwmEnable();
        lamp.setPosition(on ? LAMP_ON_POS : LAMP_OFF_POS);
    }

    public void pipelineSwitch(int index) {
        if (ll == null) return;
        ll.pipelineSwitch(index);
    }

    public LLResult getLatestResult() {
        return (ll == null) ? null : ll.getLatestResult();
    }

    /** True when a Limelight was found in the config and vision is running. */
    public boolean isAvailable() {
        return ll != null;
    }

    @Override
    public void update() {
        if (!enabled) {
            hasTarget = false;
            return;
        }

        double now = System.nanoTime() / 1e9;
        double dt = (lastFrameTimeS >= 0) ? (now - lastFrameTimeS) : 0.0;
        lastFrameTimeS = now;

        List<Ball> detections = new ArrayList<>();
        boolean fresh = false;
        LLResult result = ll.getLatestResult();
        if (result != null) {
            fresh = result.getStaleness() <= STALE_TIMEOUT_MS;
            List<LLResultTypes.DetectorResult> dets = result.getDetectorResults();
            if (dets != null) {
                for (LLResultTypes.DetectorResult d : dets) {
                    if (d.getConfidence() < DETECT_CONF) continue;
                    Ball b = new Ball();
                    b.cx = d.getTargetXDegrees();
                    b.cy = d.getTargetYDegrees();
                    b.area = d.getTargetArea();
                    b.color = d.getClassName();
                    detections.add(b);
                }
            }
        }

        matchAndUpdateTracks(detections, dt);

        int nz = Math.max(1, NUM_ZONES);
        int[] zoneCounts = new int[nz];
        List<List<Ball>> zones = new ArrayList<>();
        for (int i = 0; i < nz; i++) zones.add(new ArrayList<>());
        int count = 0;
        int lc = 0, cc = 0, rc = 0;
        for (Ball t : tracks) {
            if (t.missed != 0) continue;
            int z = zoneOf(t.cx, nz);
            zones.get(z).add(t);
            zoneCounts[z]++;
            count++;
            if (t.cx < -SIDE_CENTER_HALF_DEG) lc++;
            else if (t.cx > SIDE_CENTER_HALF_DEG) rc++;
            else cc++;
        }
        ballCount = count;
        zoneCountsOut = zoneCounts;
        leftCount = lc; centerCount = cc; rightCount = rc;

        boolean rawFound = false;
        double rawAvgSpeed = 0.0;
        if (count > 0) {
            int best = pickBestZone(zoneCounts, nz);

            int lockedCount = (lockedZone != null) ? zoneCounts[lockedZone] : 0;
            if (lockedZone == null || zoneCounts[best] > lockedZoneCount + ZONE_SWITCH_MIN_MARGIN || lockedCount == 0) {
                lockedZone = best;
            }
            int chosen = lockedZone;
            lockedZoneCount = zoneCounts[chosen];

            List<Ball> chosenBalls = zones.get(chosen);
            if (!chosenBalls.isEmpty()) {
                rawFound = true;
                double totalArea = 0, wcx = 0;
                for (Ball b : chosenBalls) { totalArea += b.area; wcx += b.cx * b.area; }
                double avgCx = (totalArea > 0) ? wcx / totalArea : chosenBalls.get(0).cx;
                tx = SMOOTHING_ALPHA * avgCx + (1 - SMOOTHING_ALPHA) * tx;

                double sp = 0;
                for (Ball b : chosenBalls) sp += Math.hypot(b.vx, b.vy);
                rawAvgSpeed = sp / chosenBalls.size();
                lostCounter = 0;
            }
        }

        if (!rawFound) {
            lostCounter++;
            if (lostCounter > LOST_TARGET_FRAMES) {
                tx = 0.0;
                lockedZone = null;
                lockedZoneCount = 0;
            }
        }

        avgZoneSpeed = rawAvgSpeed;
        zone = (lockedZone != null) ? lockedZone : -1;
        hasTarget = fresh && (rawFound || lostCounter <= LOST_TARGET_FRAMES);

        if (triggersEnabled) {
            for (Trigger t : triggers) t.evaluate();
        }
    }

    /**
     * Fullest zone, with a standing preference for {@link #PREFERRED_ZONE}: another zone has to lead
     * it by {@link #PREFERRED_ZONE_MARGIN} balls before we go there instead. Among the rest, more
     * balls wins and ties go to whichever sits closest to the middle of the frame.
     *
     * <p>A PREFERRED_ZONE outside [0, nz) turns the preference off and leaves the plain
     * most-balls-then-centre rule.
     */
    private static int pickBestZone(int[] zoneCounts, int nz) {
        double center = (nz - 1) / 2.0;
        int preferred = (PREFERRED_ZONE >= 0 && PREFERRED_ZONE < nz) ? PREFERRED_ZONE : -1;

        int best = -1;
        for (int i = 0; i < nz; i++) {
            if (i == preferred) continue;
            if (best < 0
                    || zoneCounts[i] > zoneCounts[best]
                    || (zoneCounts[i] == zoneCounts[best]
                        && Math.abs(i - center) < Math.abs(best - center))) {
                best = i;
            }
        }

        if (preferred < 0) return best;                 // preference disabled
        if (best < 0) return preferred;                 // only one zone exists
        return (zoneCounts[best] >= zoneCounts[preferred] + PREFERRED_ZONE_MARGIN) ? best : preferred;
    }

    private int zoneOf(double txDeg, int nz) {
        double frac = (txDeg / (HFOV_DEG / 2.0) + 1.0) / 2.0; // [-hfov/2,+hfov/2] -> [0,1]
        int z = (int) Math.floor(frac * nz);
        return Math.max(0, Math.min(nz - 1, z));
    }

    private void matchAndUpdateTracks(List<Ball> detections, double dt) {
        double safeDt = Math.max(dt, 1e-3);
        List<Ball> unmatched = new ArrayList<>(detections);
        for (Ball track : tracks) {
            Ball best = null;
            double bestDist = Double.MAX_VALUE;
            for (Ball d : unmatched) {
                double dist = Math.hypot(d.cx - track.cx, d.cy - track.cy);
                if (dist < bestDist) { bestDist = dist; best = d; }
            }
            if (best != null && bestDist < TRACK_MATCH_DIST_DEG) {
                double rawVx = (best.cx - track.cx) / safeDt;
                double rawVy = (best.cy - track.cy) / safeDt;
                track.vx = VELOCITY_ALPHA * rawVx + (1 - VELOCITY_ALPHA) * track.vx;
                track.vy = VELOCITY_ALPHA * rawVy + (1 - VELOCITY_ALPHA) * track.vy;
                track.cx = best.cx; track.cy = best.cy;
                track.area = best.area; track.color = best.color;
                track.missed = 0;
                unmatched.remove(best);
            } else {
                track.missed++;
            }
        }
        for (Ball d : unmatched) { d.vx = 0; d.vy = 0; d.missed = 0; tracks.add(d); }
        tracks.removeIf(t -> t.missed > MAX_MISSED_FRAMES);
    }

    public double getTx() { return tx; }
    public int getZone() { return zone; }
    public int[] getZoneCounts() { return zoneCountsOut; }
    public int getLeftCount() { return leftCount; }
    public int getCenterCount() { return centerCount; }
    public int getRightCount() { return rightCount; }
    public double getBallCount() { return ballCount; }
    public boolean hasTarget() { return hasTarget; }
    public double getAvgZoneSpeed() { return avgZoneSpeed; }

    public void onRising(String name, BooleanSupplier condition, Runnable action) {
        triggers.add(new Trigger(name, condition, action, false));
    }

    public void whileTrue(String name, BooleanSupplier condition, Runnable action) {
        triggers.add(new Trigger(name, condition, action, true));
    }

    public void removeTrigger(String name) {
        triggers.removeIf(t -> t.name.equals(name));
    }

    public void clearTriggers() {
        triggers.clear();
    }

    public void setTriggersEnabled(boolean enabled) {
        triggersEnabled = enabled;
    }

    private static class Ball {
        double cx, cy;
        double vx, vy;
        double area;
        String color;
        int missed;
    }

    private static class Trigger {
        final String name;
        final BooleanSupplier condition;
        final Runnable action;
        final boolean continuous;
        boolean last = false;

        Trigger(String name, BooleanSupplier condition, Runnable action, boolean continuous) {
            this.name = name;
            this.condition = condition;
            this.action = action;
            this.continuous = continuous;
        }

        void evaluate() {
            boolean now;
            try {
                now = condition.getAsBoolean();
            } catch (Exception e) {
                now = false;
            }
            if (continuous ? now : (now && !last)) {
                try {
                    action.run();
                } catch (Exception e) {
                    Log.w("Limelight", "trigger '" + name + "' failed: " + e.getMessage());
                }
            }
            last = now;
        }
    }

//    @Override
//    public void updateTelemetry(MultipleTelemetry telemetry) {
//        telemetry.addData("LL connected", connected);
//        telemetry.addData("LL hasTarget", hasTarget);
//        telemetry.addData("LL tx", tx);
//        telemetry.addData("LL ballCount", ballCount);
//    }
}
