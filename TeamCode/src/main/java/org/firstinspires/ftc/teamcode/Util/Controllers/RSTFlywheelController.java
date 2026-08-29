package org.firstinspires.ftc.teamcode.Util.Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * RST (pole-placement) velocity controller for the shooter flywheel.
 *
 * <p>Drop-in replacement for {@link velocityController}: same
 * {@code calculate(targetVel, currentVel, voltage)} signature, returns motor power in
 * [{@link #MIN_POWER}, {@link #MAX_POWER}].
 *
 * <h3>Plant model (from open-loop system ID, see FlywheelSysId)</h3>
 * The flywheel is a clean first-order system, velocity in encoder ticks/sec vs. motor power:
 * <pre>
 *   steady state : vel = {@link #K_DC} * power - offset     (K_DC ~ 2962 tps per unit power)
 *   dynamics     : first order, time constant {@link #TAU} ~ 0.46 s
 * </pre>
 * Discretised with a zero-order hold at the loop period {@code Ts}:
 * <pre>
 *   v(k) = a*v(k-1) + b*u(k-1),   a = exp(-Ts/TAU),   b = K_DC*(1-a)
 * </pre>
 *
 * <h3>Controller</h3>
 * Two-degree-of-freedom RST with an integrator ({@code S = 1 - q^-1}) so steady-state error is
 * zero and disturbances (a ball dragging the wheel down) are rejected. Pole placement solves
 * {@code A*S + B*R = A_cl} for a first-order plant + integrator = two closed-loop poles:
 * <pre>
 *   u(k) = u(k-1) + t0*r(k) - r0*y(k) - r1*y(k-1)
 * </pre>
 * The two poles are placed as <b>real</b> poles from {@link #TAU_CL} (tracking speed) and
 * {@link #TAU_I} (disturbance-rejection speed). Real poles => no overshoot, which matters here:
 * the flywheel accelerates fast but can barely decelerate (coast-down is Coulomb-limited to
 * ~200 tps/s), so overshoot would be slow and ugly to recover from.
 *
 * <p>Coefficients are recomputed every call from the <b>measured</b> {@code dt}, so loop jitter
 * does not detune the loop. The only carried state is the previous command and measurement, both
 * physical quantities, so the varying-gain incremental form stays well-behaved.
 */
@Config
public class RSTFlywheelController {

    // ---- Identified plant (FlywheelSysId 2026-08-29 run 113957, 11-step sweep, ~13.6 V) ----
    //
    // The drag that showed up in the 2026-08-28 run is gone. Normalized per volt, so the three runs
    // are comparable across their different battery states:
    //
    //     run      K_DC/V   ceiling/V
    //     08-26     207.9     184.7
    //     08-28     195.7     147.8   <- dragging
    //     08-29     217.5     189.5   <- fixed, and slightly better than the 08-26 baseline
    //
    // The offset came back down (618 -> 378 tps) and TAU with it (0.81 -> 0.46 s), which is what
    // removing friction looks like. The headline 2581 tps ceiling is partly the fresher battery;
    // per volt the wheel is ~2.6% better than it was on 08-26, not 33%.
    /**
     * Steady-state gain: ticks/sec per unit motor power. Voltage-normalized fit over the high-power
     * region (power >= 0.60, where we actually shoot): vel = 2962*power - 378. The full-range fit is
     * ~2838; the plant is steeper near the operating point, so we use the high-power fit.
     */
    public static double K_DC = 2962.0;
    /**
     * Open-loop spin-up time constant, seconds. TAU is power-dependent - ~0.38 at mid power, ~0.46
     * near the far-zone operating point (0.85-1.00 power). Since far is the default shooting mode,
     * TAU is set to the operating-point value, not the mid-range one.
     */
    public static double TAU = 0.46;

    // ---- Feedforward (from the same fit) used only to seed the integrator on reset ----
    /** Power per tps (~1/K_DC). */
    public static double KV_FF = 0.000338;
    /** Static/deadband power (high-power fit offset 378 tps / gain 2962). */
    public static double KS_FF = 0.128;

    /**
     * MAX ACHIEVABLE VELOCITY, tps, at {@link #V_NOMINAL} and power 1.0. Not used by the loop - it
     * is here so the number is written down. Scale it by (voltage / V_NOMINAL) for a sagged battery:
     * ~2274 tps at 12.0 V, ~2179 at 11.5 V. Any target above that is unreachable and
     * {@code Launcher.isReady()} will never go true.
     */
    public static double V_MAX_AT_NOMINAL = 2581.0;

    // ---- Closed-loop tuning knobs (the two things you actually tune) ----
    /** Tracking closed-loop time constant, seconds. Smaller = snappier spin-up. */
    public static double TAU_CL = 0.15;
    /** Disturbance-rejection (integrator) time constant, seconds. Smaller = faster RPM recovery after a shot. */
    public static double TAU_I = 0.4;

    // ---- Output / voltage ----
    public static double MAX_POWER = 1.0;
    /** Lower bound. 0 = coast only (no active braking); negative allows the motors to brake. */
    public static double MIN_POWER = -0.25;
    public static boolean USE_VOLTAGE_COMP = true;
    /** Battery voltage the plant gain K_DC was identified at (2026-08-29 run 113957 averaged ~13.6 V). */
    public static double V_NOMINAL = 13.62;

    // ---- dt guard ----
    public static double DT_MIN = 0.004;
    public static double DT_MAX = 0.05;
    public static double DT_DEFAULT = 0.02;

    private final ElapsedTime timer = new ElapsedTime();
    private double uPrev = 0.0;
    private double yPrev = 0.0;
    private boolean firstRun = true;

    /** Debug: last solved controller coefficients / poles. */
    public double lastR0, lastR1, lastT0, lastPole1, lastPole2, lastDt;

    public RSTFlywheelController() {
        reset();
    }

    /**
     * Reset integrator/history. Call when (re)starting the flywheel so it seeds from the
     * feedforward power for the current target rather than winding up from zero.
     */
    public void reset() {
        firstRun = true;
        uPrev = 0.0;
        yPrev = 0.0;
        timer.reset();
    }

    /**
     * @param targetVelocity  desired flywheel velocity, ticks/sec
     * @param currentVelocity measured flywheel velocity, ticks/sec (same signal the loop closes on)
     * @param voltage         current battery voltage
     * @return motor power, clamped to [MIN_POWER, MAX_POWER]
     */
    public double calculate(double targetVelocity, double currentVelocity, double voltage) {
        double dt = timer.seconds();
        timer.reset();
        if (dt < DT_MIN || dt > DT_MAX || Double.isNaN(dt)) dt = DT_DEFAULT;
        lastDt = dt;

        if (firstRun) {
            // Seed the integrator at the operating-point power so we don't ramp from 0.
            uPrev = clampFF(KV_FF * targetVelocity + Math.signum(targetVelocity) * KS_FF);
            yPrev = currentVelocity;
            firstRun = false;
        }

        // --- Discrete first-order plant ---
        double a = Math.exp(-dt / Math.max(TAU, 1e-3));
        double b = K_DC * (1.0 - a);
        if (Math.abs(b) < 1e-6) b = 1e-6; // guard

        // --- Desired closed-loop poles (real -> no overshoot) ---
        double p1 = Math.exp(-dt / Math.max(TAU_CL, 1e-3));
        double p2 = Math.exp(-dt / Math.max(TAU_I, 1e-3));
        double alpha1 = -(p1 + p2);
        double alpha2 = p1 * p2;
        lastPole1 = p1; lastPole2 = p2;

        // --- Pole placement: A*S + B*R = A_cl, with S = 1 - q^-1 (integrator) ---
        // A = 1 - a q^-1, B = b q^-1, R = r0 + r1 q^-1
        double r0 = (alpha1 + 1.0 + a) / b;
        double r1 = (alpha2 - a) / b;
        // T scalar for unity DC gain: T = A_cl(1)/B(1)
        double t0 = (1.0 + alpha1 + alpha2) / b;
        lastR0 = r0; lastR1 = r1; lastT0 = t0;

        // --- RST control law (incremental / velocity form) ---
        double u = uPrev + t0 * targetVelocity - r0 * currentVelocity - r1 * yPrev;

        // Voltage compensation: K_DC was measured at V_NOMINAL; scale to hold effective volts constant.
        if (USE_VOLTAGE_COMP && voltage > 6.0) {
            u *= (V_NOMINAL / voltage);
        }

        // Clamp + conditional-integration anti-windup: carry forward the *clamped* command.
        double uClamped = Math.max(MIN_POWER, Math.min(MAX_POWER, u));
        // Store the un-voltage-comp'd command as integrator state so comp doesn't accumulate.
        double uState = (USE_VOLTAGE_COMP && voltage > 6.0) ? uClamped * (voltage / V_NOMINAL) : uClamped;
        uPrev = Math.max(MIN_POWER, Math.min(MAX_POWER, uState));
        yPrev = currentVelocity;

        return uClamped;
    }

    private static double clampFF(double v) {
        return Math.max(MIN_POWER, Math.min(MAX_POWER, v));
    }
}
