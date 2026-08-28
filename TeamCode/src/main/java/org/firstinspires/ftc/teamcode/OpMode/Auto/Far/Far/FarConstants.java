package org.firstinspires.ftc.teamcode.OpMode.Auto.Far.Far;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Util.Globals.Alliance;
import org.firstinspires.ftc.teamcode.Util.Info;

@Config
public class FarConstants {

    public static int limelightPipeline = 7;
    public static int fallbackZone = 0;

    public static double zoneSpeedThreshold = 10000; //90 px/s @ 640px: 90 * 54.5/640 cred
    public static int zoneSpeedShiftPose = -1;

    /**
     * Drives with blob's RST follower instead of the position PIDs.
     *
     * <p>Live on the dashboard, so it can be turned off mid-match if it misbehaves. If the measured
     * constants do not add up the follower refuses to drive, and {@link Far} falls back to the PID on
     * its own rather than sitting still, so this failing cannot cost the auto.
     */
    public static boolean useRST = true;

    public static int cycleCount = 6;

    public static double launcherVelocity = 0;
    public static double hoodPosition = 0;
    public static double shootingTime = 500;
    public static double shootingPercentage = 0.95;
    public static double cycleIntakeUntilPercentage = 0.15;
    public static double cyclePickupSlowPercentage = 0.45;
    public static double cyclePickupSlowPower = 0.7;
    public static double rescanPercentage = 0.6;
    public static double sideShiftRightInches = 6.0; // lateral veer toward the robot's right
    public static double sideShiftLeftInches = 6.0;  // lateral veer toward the robot's left
    public static double hpIntakeUntilPercentage = 0.6;

    public static double failSafeDtTime = 725;
    public static long failSafePickupTime = 825;
    public static double zoneDecideTimeout = 750;
    public static double scoreFailSafeDtTime = 2500;
    public static double scoreSettleDelay = 250;



    public static double pickUp3XIntermediary  = 36,pickUp3YIntermediairy = 24, pickUp3HeadingIntermediary = Math.PI/2;
    public static double pickUp3X = 36,pickUp3Y= 54, pickUp3Heading = Math.PI/2;
    public Pose pickUpPose3;
    public Pose pickUpPose3Intermediary;


    public static double pickupFailsafeX = 0, pickupFailsafeY = 0, pickupFailsafeHeading = 0;
    public Pose pickupFailsafePose;





    public static double startX = 63.21, startY = 17.77, startHeading = Math.PI/2;
    public Pose startPose;

    public static double humanPlayerX = 63.90, humanPlayerY = 61.5, humanPlayerHeading = Math.PI/2;
    public Pose humanPlayerPose;

    public static double scoreX = 55, scoreY = 14, scoreHeading = 1.650;
    public Pose scorePose;



    // 3 zones, equally spaced across the far pickup lane (X 64.5 -> 27.3, step 18.6"), all at Y=58.92.
    public static double zone0X = 63.90, zone0Y = 61.5, zone0Heading = Math.PI/2;
    public static double zone1X = 48.24, zone1Y = 61.5, zone1Heading = Math.PI/2;
    public static double zone2X = 28.47, zone2Y = 61.5, zone2Heading = Math.PI/2;
    public Pose zonePose0, zonePose1, zonePose2;
    public Pose[] zonePoses;


    public static double parkX = 61.30, parkY = 31.24, parkHeading = Math.PI/2;
    public Pose parkPose;




    public void buildPaths() {
        int m = (Info.alliance == Alliance.RED) ? 1 : -1;

        startPose = new Pose(startX, startY * m, startHeading * m);
        humanPlayerPose = new Pose(humanPlayerX, humanPlayerY * m, toBlobHeading(humanPlayerHeading * m));
        scorePose = new Pose(scoreX, scoreY * m, toBlobHeading(scoreHeading * m));

        pickUpPose3Intermediary = new Pose(pickUp3XIntermediary, pickUp3YIntermediairy * m, toBlobHeading(pickUp3HeadingIntermediary * m));
        pickUpPose3 = new Pose(pickUp3X, pickUp3Y * m, toBlobHeading(pickUp3Heading * m));

        zonePose0 = new Pose(zone0X, zone0Y * m, toBlobHeading(zone0Heading * m));
        zonePose1 = new Pose(zone1X, zone1Y * m, toBlobHeading(zone1Heading * m));
        zonePose2 = new Pose(zone2X, zone2Y * m, toBlobHeading(zone2Heading * m));
        zonePoses = new Pose[]{zonePose0, zonePose1, zonePose2};

        parkPose = new Pose(parkX, parkY * m, toBlobHeading(parkHeading * m));
        pickupFailsafePose = new Pose(pickupFailsafeX, pickupFailsafeY * m, toBlobHeading(pickupFailsafeHeading * m));
    }

    private static double toBlobHeading(double fieldHeading) {
        double h = fieldHeading % (2 * Math.PI);
        if (h < 0) h += 2 * Math.PI;          // normalize to [0, 2PI)
        return (2 * Math.PI - h) % (2 * Math.PI);
    }

    public Pose zonePose(int zoneIndex) {
        return zonePoses[poseIndexFor(zoneIndex)];
    }

    public int poseIndexFor(int zoneIndex) {
        if (Info.alliance == Alliance.RED && zoneIndex >= 0 && zoneIndex < zonePoses.length) {
            zoneIndex = zonePoses.length - 1 - zoneIndex;
        }
        if (zoneIndex < 0 || zoneIndex >= zonePoses.length) zoneIndex = fallbackZone;
        return zoneIndex;
    }

    public double getZoneSpeedThreshold() { return zoneSpeedThreshold; }

    public double getLauncherVelocity() { return launcherVelocity; }
    public double getHoodPosition() { return hoodPosition; }
    public double getShootingTime() { return shootingTime; }
    public double getShootingPercentage() { return shootingPercentage; }
    public double getCycleIntakeUntilPercentage() { return cycleIntakeUntilPercentage; }
    public double getCyclePickupSlowPercentage() { return cyclePickupSlowPercentage; }
    public double getCyclePickupSlowPower() { return cyclePickupSlowPower; }
    public double getRescanPercentage() { return rescanPercentage; }
    public double getSideShiftRightInches() { return sideShiftRightInches; }
    public double getSideShiftLeftInches() { return sideShiftLeftInches; }
    public double getHpIntakeUntilPercentage() { return hpIntakeUntilPercentage; }
    public double getFailSafeDtTime() { return failSafeDtTime; }
    public double getScoreFailSafeDtTime() { return scoreFailSafeDtTime; }
    public double getScoreSettleDelay() { return scoreSettleDelay; }
    public long getFailSafePickupTime() { return failSafePickupTime; }
    public double getZoneDecideTimeout() { return zoneDecideTimeout; }
    public int getCycleCount() { return cycleCount; }
}
