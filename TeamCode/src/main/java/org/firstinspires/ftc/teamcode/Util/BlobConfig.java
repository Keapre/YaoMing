package org.firstinspires.ftc.teamcode.Util;

import com.acmerobotics.dashboard.config.Config;
import com.blob.BlobParams;
import com.blob.localization.GoBildaPinpointDriver;
import com.blob.localization.Localizer;
import com.blob.localization.OctoQuadFWv3Localizer;
import com.blob.localization.OctoQuadLocalizer;
import com.blob.localization.OctoQuadTwoWheelLocalizer;
import com.blob.localization.PinpointLocalizer;
import com.blob.localization.SdkPinpointLocalizer;
import com.blob.tuning.TuningConfig;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

/**
 * Our tuned blob numbers, and the bridge between them and the library's {@link BlobParams}.
 *
 * <p>The library takes config as an instance you hand to the constructor, while FtcDashboard can
 * only edit static fields. So the statics live here and {@link #sync} copies them into the live
 * params object every loop, which is what keeps dashboard sliders working. {@code Blob.update()}
 * re-reads the gains from params each loop anyway, so a value edited mid-run takes effect
 * immediately, exactly like the old static {@code BlobConstants} did.
 *
 * <p>Replaces {@code org.firstinspires.ftc.teamcode.blob.constants.BlobConstants}. Values carried
 * over unchanged, so behaviour is identical to the in-project follower.
 */
@Config
public class BlobConfig {

    /**
     * Which odometry hardware to localize from. Every one of these is two pods plus an IMU; they
     * differ in whose IMU it is and who fuses it.
     *
     * <p>Changing this is not enough on its own. Each mode needs a matching entry in the robot
     * config, and the OctoQuad modes need their pod geometry filled in below.
     */
    public enum LocalizerMode {
        /**
         * goBILDA Pinpoint via the SDK's own driver. Config entry: "goBILDA Pinpoint Odometry
         * Computer". This is what our robot runs, and it needs nothing changed on the hub.
         */
        PINPOINT_SDK,

        /**
         * goBILDA Pinpoint via blob's bundled driver. Config entry: "blob Pinpoint (goBILDA)".
         * Only for SDKs older than 10.2, which have no built-in Pinpoint driver.
         */
        PINPOINT_BUNDLED,

        /**
         * OctoQuad MK2 via the SDK's built-in driver. Config entry: "OctoQuadFTC".
         *
         * <p>If your board's firmware minor revision is newer than the SDK driver expects, this
         * puts a permanent warning banner on the Driver Station even though nothing is wrong. Use
         * {@link #OCTOQUAD_MK2_FWV3} instead in that case.
         */
        OCTOQUAD_MK2,

        /**
         * OctoQuad MK2 via blob's vendored copy of DigitalChickenLabs' firmware v3 driver.
         * Config entry: "blob OctoQuad (FW v3)". This is what our robot runs.
         *
         * <p>Same board, same behaviour as {@link #OCTOQUAD_MK2}. The only difference is that a
         * newer-minor firmware revision is logged rather than raised as a Driver Station banner,
         * because a newer minor revision is backward compatible and nothing is actually broken.
         */
        OCTOQUAD_MK2_FWV3,

        /**
         * Any OctoQuad, including MK1, with heading from the Control Hub IMU. blob integrates the
         * pods itself. Needs an "imu" in the config, the hub orientation, and the pod offsets in
         * INCHES.
         */
        OCTOQUAD_TWO_WHEEL
    }

    /** Which of the above to build. The only line to change when swapping odometry hardware. */
    public static LocalizerMode localizerMode = LocalizerMode.OCTOQUAD_MK2_FWV3;

    // HardwareMap
    public static String leftFrontName = "fl";
    public static String leftBackName = "bl";
    public static String rightFrontName = "fr";
    public static String rightBackName = "br";
    public static String pinpointName = "pinpoint";

    // In-position thresholds
    public static double hDefTresh = Math.toRadians(3);
    public static double xDefTresh = 1;
    public static double yDefTresh = 1;
    public static double defaultVelocityThresh = 4;
    public static double defaultTransThresh = 2;

    // PID. kI and hI are ignored: Blob.update() forces the integral term to zero.
    public static double kP = 0.12, kI = 0, kD = 0.01;
    public static double hP = 1.2, hI = 0, hD = 0.11;

    // Deceleration, from the deceleration tuner
    public static double xDeceleration = 80, yDeceleration = 90;

    public static double lateralMultiplier = 1.4;
    public static double voltageConstant = 12;

    // Localization
    //
    // We run an OctoQuad MK2, whose geometry is the TCP offset in millimetres, so that is what is
    // set here and it goes to the board unchanged. It used to be derived from Pinpoint-style pod
    // offsets in inches, which meant every value made a round trip of mm -> inches -> per-pod
    // convention -> mm with an axis swap and a sign flip. Three chances to be wrong for no benefit,
    // since nothing here runs a Pinpoint.
    //
    // These two numbers reproduce exactly what the derivation was producing, so behaviour is
    // unchanged. Whether they are RIGHT is a separate question: run blob: Rotation Offset Tuner,
    // which measures the offset from the circle the pose traces while the robot spins, and does not
    // care which convention anything was originally written in.
    public static double octoQuadTcpOffsetMM_X = -104.60146411045879;
    public static double octoQuadTcpOffsetMM_Y = -50.98700462712526;

    // Kept for the Pinpoint localizers, which we do not use. Not the source of the TCP offsets.
    public static double yOffset = 104.60146411045879 / 25.4;
    public static double xOffset = 50.98700462712526 / 25.4;
    public static GoBildaPinpointDriver.EncoderDirection xPodDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection yPodDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.GoBildaOdometryPods podType = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    /**
     * Aims the PID at the glide-predicted position instead of the current one. Was a static on the
     * old Blob; it lives on the instance now, so Robot.update() pushes it across.
     */
    public static boolean usePredictiveDecel = false;

    // OctoQuad geometry. Ignored unless localizerMode is one of the OCTOQUAD_ ones.
    public static String octoQuadName = "octoquad";
    public static int octoQuadPortX = 0;   // forward-reading pod
    public static int octoQuadPortY = 1;   // strafe-reading pod
    /**
     * Counts per mm of pod travel. The Quickstart guide recommends MEASURING this by pushing the
     * robot a known distance rather than trusting the spec figure, because it depends on the wheel
     * as built, not on paper. This is the goBILDA 4-bar spec value; swingarm is 13.26291192.
     */
    public static double octoQuadCountsPerMM = 19.89436789;

    /**
     * Pod directions. Required: with the robot at 0 degrees, pushing it FORWARD must make the X pod
     * count up, and pushing it LEFT must make the Y pod count up. Expect to flip at least one.
     */
    public static GoBildaPinpointDriver.EncoderDirection octoQuadXDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    public static GoBildaPinpointDriver.EncoderDirection octoQuadYDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;

    /**
     * Corrects the IMU reporting more or less rotation than actually happened. A wrong scalar does
     * not cause drift; it makes every turn come out the wrong size, which shows up as heading
     * skewing further and further out the more the robot spins.
     *
     * <p><b>Still a placeholder, not a measurement.</b> Run {@code blob: Heading Scalar Tuner},
     * turn the robot by hand through ten full turns, and put the number it gives you here.
     */
    public static double octoQuadImuHeadingScalar = 1.02;

    /** Hardware velocity averaging window, ms. Longer is smoother with more lag. */
    public static int octoQuadVelocityIntervalMS = 25;

    /**
     * How long to wait for the MK2 IMU to finish calibrating at init, ms.
     *
     * <p>Matches the library default. Giving up early means using the IMU before its zero-rate bias
     * is calibrated, and that bias is what stops heading drifting. Note this value is synced into
     * the params every loop, so it wins over whatever the library ships.
     */
    public static long octoQuadInitTimeoutMs = 5000;

    public static boolean octoQuadInvertHeading = false;

    /**
     * Pinpoint-style pod offsets, in inches. Only read by the Pinpoint localizers, which we do not
     * use, so changing them does nothing on this robot. The OctoQuad's geometry is
     * {@link #octoQuadTcpOffsetMM_X} and {@link #octoQuadTcpOffsetMM_Y} above.
     */
    /** Two-wheel mode only. Pod offsets in INCHES, same measurements the Pinpoint uses. */
    public static double octoQuadTicksPerInch = 19.89436789 * 25.4;

    // -----------------------------------------------------------------------------------
    // Measured drivetrain constants, from blob: Autotune. Paste its output here.
    //
    // All zero means not measured, and Blob.State.RST refuses to drive rather than following
    // a model of nothing. State.PID ignores them entirely, so leaving them empty costs nothing.
    // -----------------------------------------------------------------------------------
    public static double forwardKS = 1.005556977145929;
    public static double forwardKV = 0.1531052616714671;
    public static double forwardKA = 0.020939451511571863;

    public static double lateralKS = 1.8496674201113557;
    public static double lateralKV = 0.16649036357220193;
    public static double lateralKA = 0.018977932127223332;

    public static double angularKS = 1.2282686278472141;
    public static double angularKV = 1.984012525580037;
    public static double angularKA = 0.1354479467776826;

    /**
     * How fast an RST velocity loop should converge, seconds.
     *
     * <p>0.08 is the four-loop floor rather than anything about this drivetrain: half the slowest
     * axis tau would be 0.068, and no pole placement beats the rate the loop runs at. So the loop
     * time is the limit here, not the robot.
     */
    public static double rstTauClosedLoop = 0.08;
    /** Its integral pole. Disturbances only: it does not slow tracking down. */
    public static double rstTauIntegral = 0.32;
    /** Final approach time constant, seconds. Larger settles gentler, smaller arrives sooner. */
    public static double rstApproachTau = 0.25;
    /** How fast a turn to ask for, as opposed to how fast to meet it. */
    public static double rstHeadingTau = 0.3;
    /**
     * Speed cap for the RST follower, in/s. 0 would derive it from the constants and the battery.
     *
     * <p>Seventy is what the constants say this drivetrain can hold, so this is the whole of it.
     *
     * <p>The thing to watch is the stop, not the speed. The profile brakes on xDeceleration and
     * yDeceleration, and those are still the library's defaults rather than anything measured here.
     * At 80 in/s^2 a stop from 70 needs 31 inches; if the real figure is nearer 50 it needs 49, and
     * the difference arrives as overshoot at a pickup pose. The motors are not the limit either way,
     * they can brake at over a thousand, so what those numbers describe is the carpet.
     *
     * <p>blob: Deceleration Tuner measures them. It is the one input the autotune does not produce.
     */
    public static double rstMaxVelocity = 70;

    /** Turn rate cap, rad/s. 0 derives it from the constants and the battery. */
    public static double rstMaxAngularVelocity = 0;
    /** How much of the achievable speed to use. The rest is headroom for the loop. */
    public static double rstSpeedFraction = 0.85;

    // Curves
    public static double splineLookahead = 6.0;
    public static int splineSamples = 64;

    /**
     * Records paths for {@code pusher visualiser}. Only does anything on the dev artifact; the
     * competition one has no recorder to switch on.
     */
    public static boolean recordTrace = true;

    /** Builds the localizer for {@link #localizerMode}. */
    public static Localizer localizer(HardwareMap hardwareMap, BlobParams params) {
        switch (localizerMode) {
            case PINPOINT_BUNDLED:
                return new PinpointLocalizer(hardwareMap, params);
            case OCTOQUAD_MK2:
                return new OctoQuadLocalizer(hardwareMap, params);
            case OCTOQUAD_MK2_FWV3:
                return new OctoQuadFWv3Localizer(hardwareMap, params);
            case OCTOQUAD_TWO_WHEEL:
                return new OctoQuadTwoWheelLocalizer(hardwareMap, params);
            case PINPOINT_SDK:
            default:
                return new SdkPinpointLocalizer(hardwareMap, params);
        }
    }

    /** A fresh params object with everything above applied. */
    public static BlobParams params() {
        BlobParams p = new BlobParams();
        sync(p);
        return p;
    }

    /** Pushes the current static values into an existing params object. Call once per loop. */
    public static void sync(BlobParams p) {
        p.leftFrontName = leftFrontName;
        p.leftBackName = leftBackName;
        p.rightFrontName = rightFrontName;
        p.rightBackName = rightBackName;
        p.pinpointName = pinpointName;

        p.headingThreshRad = hDefTresh;
        p.xThresh = xDefTresh;
        p.yThresh = yDefTresh;
        p.defaultVelocityThresh = defaultVelocityThresh;
        p.defaultTransThresh = defaultTransThresh;

        p.kP = kP; p.kI = kI; p.kD = kD;
        p.hP = hP; p.hI = hI; p.hD = hD;

        p.forwardKS = forwardKS; p.forwardKV = forwardKV; p.forwardKA = forwardKA;
        p.lateralKS = lateralKS; p.lateralKV = lateralKV; p.lateralKA = lateralKA;
        p.angularKS = angularKS; p.angularKV = angularKV; p.angularKA = angularKA;
        p.rstTauClosedLoop = rstTauClosedLoop;
        p.rstTauIntegral = rstTauIntegral;
        p.rstApproachTau = rstApproachTau;
        p.rstHeadingTau = rstHeadingTau;
        p.rstMaxVelocity = rstMaxVelocity;
        p.rstMaxAngularVelocity = rstMaxAngularVelocity;
        p.rstSpeedFraction = rstSpeedFraction;

        p.xDeceleration = xDeceleration;
        p.yDeceleration = yDeceleration;
        p.lateralMultiplier = lateralMultiplier;

        p.xOffset = xOffset;
        p.yOffset = yOffset;
        p.xPodDirection = xPodDirection;
        p.yPodDirection = yPodDirection;
        p.podType = podType;

        p.octoQuadName = octoQuadName;
        p.octoQuadPortX = octoQuadPortX;
        p.octoQuadPortY = octoQuadPortY;
        p.octoQuadCountsPerMM_X = octoQuadCountsPerMM;
        p.octoQuadCountsPerMM_Y = octoQuadCountsPerMM;
        p.octoQuadXDirection = octoQuadXDirection;
        p.octoQuadYDirection = octoQuadYDirection;
        p.octoQuadImuHeadingScalar = octoQuadImuHeadingScalar;
        p.octoQuadVelocityIntervalMS = octoQuadVelocityIntervalMS;
        p.octoQuadInitTimeoutMs = octoQuadInitTimeoutMs;
        p.octoQuadInvertHeading = octoQuadInvertHeading;
        // Set directly rather than derived from xOffset/yOffset. One source of truth, in the units
        // the board actually takes.
        p.octoQuadTcpOffsetMM_X = octoQuadTcpOffsetMM_X;
        p.octoQuadTcpOffsetMM_Y = octoQuadTcpOffsetMM_Y;
        p.octoQuadTicksPerInch = octoQuadTicksPerInch;
        // The two-wheel localizer reuses the Pinpoint's pod geometry, so there is one set to tune.
        p.parallelPodYOffset = xOffset;
        p.perpendicularPodXOffset = yOffset;

        p.splineLookahead = splineLookahead;
        p.splineSamples = splineSamples;
        p.recordTrace = recordTrace;
    }

    /**
     * Points the library's bundled tuning OpModes at these values instead of stock defaults.
     * Without this they would tune against a robot that is not ours.
     */
    @OpModeRegistrar
    public static void registerTuning(OpModeManager manager) {
        TuningConfig.useParams(BlobConfig::params);
        TuningConfig.useLocalizer(BlobConfig::localizer);
    }
}
