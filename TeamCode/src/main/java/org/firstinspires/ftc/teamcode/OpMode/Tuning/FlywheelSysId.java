package org.firstinspires.ftc.teamcode.OpMode.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Util.BlobConfig;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;
import java.util.Locale;

/**
 * Open-loop system-identification logger for the shooter flywheel.
 *
 * <p>Drives {@code shooter1}/{@code shooter2} with a fixed sequence of RAW power steps (NO closed-loop
 * controller) and records the flywheel velocity so a plant model can be fit offline. The velocity is
 * read from the exact same signal the real controller closes on: the flywheel encoder wired into the
 * front-left drivetrain port ("fl"), the way {@link org.firstinspires.ftc.teamcode.Hardware.Outtake.Launcher}
 * gets it via {@code blob.returnFrVelocity()}. The {@code fl} motor is left un-powered (BRAKE), so the
 * robot does not drive — only its encoder input is read.
 *
 * <p>Profile: each power in {@link #POWER_SEQUENCE} is held for {@link #DWELL_SECONDS} (long enough to
 * reach steady state), then a final {@code power = 0} coast-down is captured for {@link #COAST_SECONDS}
 * (gives the time constant and friction from the spin-down). Samples are written at ~{@link #SAMPLE_MS}
 * to a CSV under {@code /sdcard/FIRST/flywheel-sysid/}.
 *
 * <p>CSV columns: {@code t_s, power, vel_tps, voltage_v}.
 *
 * <p><b>Run it:</b> put the robot on a stand or somewhere the flywheel can spin freely, select this
 * opmode, press start, and leave it alone until it reports "DONE". Re-run for a fresh file if a run
 * looks bad. Press stop at any time to end early — whatever was logged is flushed and kept.
 */
@Config
@TeleOp(name = "Flywheel SysID Logger", group = "tuning")
public class FlywheelSysId extends LinearOpMode {

    /** Raw open-loop power levels, each held for {@link #DWELL_SECONDS}. Up then down excites the plant both ways. */
    public static double[] POWER_SEQUENCE = {0.20, 0.35, 0.50, 0.65, 0.80, 0.50, 0.20};
    /** Seconds to hold each power level. Must exceed the spin-up settling time (flywheel tau is well under a second). */
    public static double DWELL_SECONDS = 2.5;
    /** Seconds of final coast-down (power forced to 0) to capture the passive spin-down. */
    public static double COAST_SECONDS = 4.0;
    /** Sample/log period in milliseconds (~100 Hz at 10 ms). */
    public static double SAMPLE_MS = 10;

    /** Match Launcher's motor directions so logged power sign == what the real controller commands. */
    public static boolean SHOOTER1_REVERSED = true;   // Launcher: shooter1 = REVERSE
    public static boolean SHOOTER2_REVERSED = false;  // Launcher: shooter2 = FORWARD
    /**
     * Software sign on the logged/used velocity (NOT a setDirection, since the encoder shares the
     * shooter1 port). Set this so the signed velocity is POSITIVE while the wheel spins the shooting
     * direction. Use the DIRECTION CHECK phase to dial it in.
     */
    public static boolean FL_ENCODER_REVERSED = false;

    /** Open-loop power used during the interactive DIRECTION CHECK phase (before the logged sweep). */
    public static double DIR_CHECK_POWER = 0.35;

    private static final String OUT_DIR = "/sdcard/FIRST/flywheel-sysid";

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Fresh velocity every loop: clear the bulk cache ourselves each iteration.
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule h : hubs) h.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        DcMotorEx shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        DcMotorEx shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        // Flywheel velocity encoder is on the shooter1 port (the same motor we drive). Its sign
        // follows shooter1's direction; FL_ENCODER_REVERSED is applied in software on top of that.
        DcMotorEx flEncoder = hardwareMap.get(DcMotorEx.class, "shooter1");
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();

        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // FLOAT so the coast-down segment is a true passive spin-down. (BRAKE here would corrupt TAU.)
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // NOTE: flEncoder shares the shooter1 port, so its sign is governed by shooter1's setDirection.
        // We do NOT call setDirection on it (that would fight SHOOTER1_REVERSED on the same port).
        // FL_ENCODER_REVERSED is applied as a software sign on the logged velocity instead, so the
        // motor direction and the encoder sign can be chosen independently. Directions are (re)applied
        // every loop from the live flags so dashboard edits take effect immediately.

        // Build the schedule: each power for DWELL_SECONDS, then 0 for COAST_SECONDS.
        int n = (POWER_SEQUENCE == null) ? 0 : POWER_SEQUENCE.length;
        double[] segPower = new double[n + 1];
        double[] segEnd = new double[n + 1]; // cumulative end time of each segment
        double cum = 0;
        for (int i = 0; i < n; i++) {
            segPower[i] = POWER_SEQUENCE[i];
            cum += DWELL_SECONDS;
            segEnd[i] = cum;
        }
        segPower[n] = 0.0;               // coast-down segment
        cum += COAST_SECONDS;
        segEnd[n] = cum;
        double totalDuration = cum;

        BufferedWriter writer = null;
        File file = null;
        try {
            File dir = new File(OUT_DIR);
            if (!dir.exists()) dir.mkdirs();
            String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
            file = new File(dir, "flywheel_sysid_" + stamp + ".csv");
            writer = new BufferedWriter(new FileWriter(file));
            writer.write("t_s,power,vel_tps,voltage_v\n");
        } catch (IOException e) {
            telemetry.addLine("FAILED to open log file: " + e.getMessage());
            telemetry.update();
        }

        telemetry.addLine("Flywheel SysID ready.");
        telemetry.addData("Log file", file == null ? "<none>" : file.getAbsolutePath());
        telemetry.addData("Steps", n + " x " + DWELL_SECONDS + "s + coast " + COAST_SECONDS + "s");
        telemetry.addData("Total run", String.format(Locale.US, "%.1f s", totalDuration));
        telemetry.addLine("Put flywheel where it can spin freely, then press START.");
        telemetry.update();

        waitForStart();

        // ---- DIRECTION CHECK (interactive, before the logged sweep) ----
        // Jog the shooters and set the three flags LIVE in the dashboard until:
        //   1) the wheel spins the SHOOTING direction (throws a ball out) under positive power, and
        //   2) the signed velocity below reads POSITIVE while it does.
        // Then press B to start the logged sweep.
        boolean startSweep = false;
        boolean prevB = false;
        while (opModeIsActive() && !startSweep) {
            for (LynxModule h : hubs) h.clearBulkCache();
            applyMotorDirections(shooter1, shooter2);

            double jogBoth = gamepad1.a ? DIR_CHECK_POWER : 0.0;
            // Per-motor jog to check each one alone: X = shooter1 only, Y = shooter2 only.
            double s1p = gamepad1.x ? DIR_CHECK_POWER : jogBoth;
            double s2p = gamepad1.y ? DIR_CHECK_POWER : jogBoth;
            shooter1.setPower(s1p);
            shooter2.setPower(s2p);

            double raw = flEncoder.getVelocity();
            double signed = raw * (FL_ENCODER_REVERSED ? -1.0 : 1.0);
            boolean spinning = Math.abs(signed) > 30;

            telemetry.addLine("== DIRECTION CHECK ==");
            telemetry.addLine("A = spin BOTH, X = shooter1 only, Y = shooter2 only.");
            telemetry.addLine("GOAL: wheel shoots OUT and 'signed vel' is POSITIVE.");
            telemetry.addLine("- wheel spins wrong way / a motor fights -> flip that SHOOTERx_REVERSED");
            telemetry.addLine("- wheel shoots right but signed vel NEGATIVE -> flip FL_ENCODER_REVERSED");
            telemetry.addData("raw vel", String.format(Locale.US, "%.1f", raw));
            telemetry.addData("signed vel", String.format(Locale.US, "%.1f", signed));
            telemetry.addData("sign OK?", !spinning ? "spin it up" : (signed > 0 ? "YES (positive)" : "NO -> flip FL_ENCODER_REVERSED"));
            telemetry.addData("SHOOTER1_REVERSED", SHOOTER1_REVERSED);
            telemetry.addData("SHOOTER2_REVERSED", SHOOTER2_REVERSED);
            telemetry.addData("FL_ENCODER_REVERSED", FL_ENCODER_REVERSED);
            telemetry.addLine("Press B to START the logged sweep.");
            telemetry.update();

            boolean bb = gamepad1.b;
            if (bb && !prevB) startSweep = true;
            prevB = bb;
        }
        shooter1.setPower(0);
        shooter2.setPower(0);
        double encSign = FL_ENCODER_REVERSED ? -1.0 : 1.0;

        ElapsedTime total = new ElapsedTime();
        ElapsedTime sinceSample = new ElapsedTime();
        int samples = 0;
        int seg = 0;

        while (opModeIsActive() && total.seconds() <= totalDuration) {
            for (LynxModule h : hubs) h.clearBulkCache();

            double t = total.seconds();
            while (seg < segEnd.length - 1 && t > segEnd[seg]) seg++;
            double power = segPower[seg];

            applyMotorDirections(shooter1, shooter2);
            shooter1.setPower(power);
            shooter2.setPower(power);

            if (sinceSample.milliseconds() >= SAMPLE_MS) {
                sinceSample.reset();
                double vel = flEncoder.getVelocity() * encSign;
                double volts = battery.getVoltage();
                if (writer != null) {
                    try {
                        writer.write(String.format(Locale.US, "%.4f,%.4f,%.2f,%.3f%n", t, power, vel, volts));
                        samples++;
                        if ((samples % 50) == 0) writer.flush();
                    } catch (IOException ignored) { }
                }
                telemetry.addData("phase", (seg < n) ? ("step " + (seg + 1) + "/" + n) : "COAST-DOWN");
                telemetry.addData("t", String.format(Locale.US, "%.2f / %.1f s", t, totalDuration));
                telemetry.addData("power", String.format(Locale.US, "%.3f", power));
                telemetry.addData("vel (tps)", String.format(Locale.US, "%.1f", vel));
                telemetry.addData("voltage", String.format(Locale.US, "%.2f", volts));
                telemetry.addData("samples", samples);
                telemetry.update();
            }
        }

        shooter1.setPower(0);
        shooter2.setPower(0);

        if (writer != null) {
            try { writer.flush(); writer.close(); } catch (IOException ignored) { }
        }

        while (opModeIsActive()) {
            telemetry.addLine("DONE.");
            telemetry.addData("samples logged", samples);
            telemetry.addData("file", file == null ? "<none>" : file.getAbsolutePath());
            telemetry.addLine("Pull the CSV and hand it over. Press STOP.");
            telemetry.update();
            sleep(200);
        }
    }

    /** (Re)apply the motor directions from the live flags. The encoder sign is handled separately. */
    private void applyMotorDirections(DcMotorEx shooter1, DcMotorEx shooter2) {
        shooter1.setDirection(SHOOTER1_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(SHOOTER2_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }
}
