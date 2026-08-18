package org.firstinspires.ftc.teamcode.Util;

import com.acmerobotics.dashboard.config.Config;
import com.blob.BlobParams;
import com.blob.localization.GoBildaPinpointDriver;
import com.blob.localization.SdkPinpointLocalizer;
import com.blob.tuning.TuningConfig;
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

    // Curves
    public static double splineLookahead = 6.0;
    public static int splineSamples = 64;

    /**
     * Records paths for {@code pusher visualiser}. Only does anything on the dev artifact; the
     * competition one has no recorder to switch on.
     */
    public static boolean recordTrace = false;

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
        TuningConfig.useLocalizer(SdkPinpointLocalizer::new);
    }
}
