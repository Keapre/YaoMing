package org.firstinspires.ftc.teamcode.Util;

import com.acmerobotics.dashboard.config.Config;
import com.blob.BlobParams;
import com.blob.localization.GoBildaPinpointDriver;
import com.blob.localization.Localizer;
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
         * OctoQuad MK2, fusing both pods with its own onboard IMU. Config entry: "OctoQuad".
         * Needs octoQuadPortX/Y, the counts per mm, and the TCP offsets in MILLIMETRES.
         */
        OCTOQUAD_MK2,

        /**
         * Any OctoQuad, including MK1, with heading from the Control Hub IMU. blob integrates the
         * pods itself. Needs an "imu" in the config, the hub orientation, and the pod offsets in
         * INCHES.
         */
        OCTOQUAD_TWO_WHEEL
    }

    /** Which of the above to build. The only line to change when swapping odometry hardware. */
    public static LocalizerMode localizerMode = LocalizerMode.PINPOINT_SDK;

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
    public static double kP = 0.05, kI = 0, kD = 0.006;
    public static double hP = 1.2, hI = 0, hD = 0.11;

    // Deceleration, from the deceleration tuner
    public static double xDeceleration = 80, yDeceleration = 90;

    public static double lateralMultiplier = 1.4;
    public static double voltageConstant = 12;

    // Localization
    public static double xOffset = -3.24803;
    public static double yOffset = 4.30527;
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
    public static double octoQuadCountsPerMM = 19.89436789;   // goBILDA 4-bar; swingarm is 13.26291192
    /** MK2 only, and in MILLIMETRES, unlike xOffset/yOffset above which are inches. */
    public static double octoQuadTcpOffsetMM_X = -82.5;
    public static double octoQuadTcpOffsetMM_Y = 109.35;
    public static boolean octoQuadInvertHeading = false;
    /** Two-wheel mode only. Pod offsets in INCHES, same measurements the Pinpoint uses. */
    public static double octoQuadTicksPerInch = 19.89436789 * 25.4;

    // Curves
    public static double splineLookahead = 6.0;
    public static int splineSamples = 64;

    /**
     * Records paths for {@code pusher visualiser}. Only does anything on the dev artifact; the
     * competition one has no recorder to switch on.
     */
    public static boolean recordTrace = false;

    /** Builds the localizer for {@link #localizerMode}. */
    public static Localizer localizer(HardwareMap hardwareMap, BlobParams params) {
        switch (localizerMode) {
            case PINPOINT_BUNDLED:
                return new PinpointLocalizer(hardwareMap, params);
            case OCTOQUAD_MK2:
                return new OctoQuadLocalizer(hardwareMap, params);
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
        p.octoQuadTcpOffsetMM_X = octoQuadTcpOffsetMM_X;
        p.octoQuadTcpOffsetMM_Y = octoQuadTcpOffsetMM_Y;
        p.octoQuadInvertHeading = octoQuadInvertHeading;
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
