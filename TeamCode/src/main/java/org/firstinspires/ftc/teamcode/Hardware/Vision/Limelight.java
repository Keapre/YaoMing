package org.firstinspires.ftc.teamcode.Hardware.Vision;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.Hardware.Module;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

@Config
public class Limelight implements Module {

    public static int IDX_TX = 0, IDX_TY = 1, IDX_COUNT = 2, IDX_HAS_TARGET = 3;
    public static int IDX_BX = 4, IDX_BY = 5, IDX_BW = 6, IDX_BH = 7;
    public static int IDX_ZONE = 1;


    public static int IN_ROBOT_X = 0, IN_ROBOT_Y = 1, IN_ROBOT_HEADING = 2;
    public static int IN_ROBOT_VX = 3, IN_ROBOT_VY = 4;

    public static double STALE_TIMEOUT_MS = 250;

    private final Robot robot;
    private final Limelight3A ll;
    private final double[] pyInputs = new double[5];

    private double tx, ty, ballCount, bx, by, bw, bh;
    private boolean hasTarget;
    private double[] raw = new double[0];

    private final List<Trigger> triggers = new ArrayList<>();
    private boolean triggersEnabled = true;

    public Limelight(Robot robot) {
        this.robot = robot;
        ll = robot.hw.get(Limelight3A.class, "limelight");
        ll.start();
    }

    private void updatePythonInputs() {
        if (robot.blob == null || robot.blob.odo == null) return;
        pyInputs[IN_ROBOT_X] = robot.blob.odo.getX();
        pyInputs[IN_ROBOT_Y] = robot.blob.odo.getY();
        pyInputs[IN_ROBOT_HEADING] = robot.blob.odo.getHeading();
        pyInputs[IN_ROBOT_VX] = robot.blob.getVelocityX();
        pyInputs[IN_ROBOT_VY] = robot.blob.getVelocityY();
        ll.updatePythonInputs(pyInputs);
    }

    public void pipelineSwitch(int index) {
        ll.pipelineSwitch(index);
    }

    public LLResult getLatestResult() {
        return ll.getLatestResult();
    }

    @Override
    public void update() {
        updatePythonInputs();

        boolean fresh = false;
        LLResult result = ll.getLatestResult();
        if (result != null) {
            double[] py = result.getPythonOutput();
            if (py != null && py.length > IDX_HAS_TARGET) {
                raw = py;
                tx = at(py, IDX_TX);
                ty = at(py, IDX_TY);
                ballCount = at(py, IDX_COUNT);
                bx = at(py, IDX_BX);
                by = at(py, IDX_BY);
                bw = at(py, IDX_BW);
                bh = at(py, IDX_BH);
                hasTarget = at(py, IDX_HAS_TARGET) > 0.5;
                fresh = result.getStaleness() <= STALE_TIMEOUT_MS;
            }
        }

        if (!fresh) {
            hasTarget = false;
        }
        if (triggersEnabled) {
            for (Trigger t : triggers) t.evaluate();
        }
    }

    private static double at(double[] a, int i) {
        return (i >= 0 && i < a.length) ? a[i] : 0.0;
    }

    public double getTx() { return tx; }
    public double getTy() { return ty; }
    public int getZone() { return (int) Math.round(at(raw, IDX_ZONE)); }
    public double getBallCount() { return ballCount; }
    public boolean hasTarget() { return hasTarget; }
    public double getBoxX() { return bx; }
    public double getBoxY() { return by; }
    public double getBoxW() { return bw; }
    public double getBoxH() { return bh; }
    public double[] getRawOutput() { return raw; }

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
