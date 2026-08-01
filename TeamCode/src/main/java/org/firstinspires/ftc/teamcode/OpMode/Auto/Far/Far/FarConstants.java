package org.firstinspires.ftc.teamcode.OpMode.Auto.Far.Far;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Util.Globals.Alliance;
import org.firstinspires.ftc.teamcode.Util.Info;

@Config
public class FarConstants {

    public static int limelightPipeline = 7;
    public static int fallbackZone = 0;

    public static double zoneSpeedThreshold = 7.66; //90 px/s @ 640px: 90 * 54.5/640 cred
    public static int zoneSpeedShiftPose = -1;

    public static int cycleCount = 6;

    public static double launcherVelocity = 0;
    public static double hoodPosition = 0;
    public static double shootingTime = 650;
    public static double shootingPercentage = 0.95;
    public static double cycleIntakeUntilPercentage = 0.15;
    public static double cyclePickupSlowPercentage = 0.7;
    public static double cyclePickupSlowPower = 0.7;
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





    public static double startX = 63.94, startY = 14.93, startHeading = Math.PI/2;
    public Pose startPose;

    public static double humanPlayerX = 63, humanPlayerY = 61.4, humanPlayerHeading = Math.PI/2;
    public Pose humanPlayerPose;

    public static double scoreX = 59.64, scoreY = 13.73, scoreHeading = -4.5250;
    public Pose scorePose;



    public static double zone0X = 63.0, zone0Y = 58.92, zone0Heading = Math.PI/2;
    public static double zone1X = 57.9, zone1Y = 58.92, zone1Heading = Math.PI/2;
    public static double zone2X = 52.8, zone2Y = 58.92, zone2Heading = Math.PI/2;
    public static double zone3X = 47.7, zone3Y = 58.92, zone3Heading = Math.PI/2;
    public static double zone4X = 42.6, zone4Y = 58.92, zone4Heading = Math.PI/2;
    public static double zone5X = 37.5, zone5Y = 58.92, zone5Heading = Math.PI/2;
    public static double zone6X = 32.4, zone6Y = 58.92, zone6Heading = Math.PI/2;
    public static double zone7X = 27.3, zone7Y = 58.92, zone7Heading = Math.PI/2;
    public Pose zonePose0, zonePose1, zonePose2, zonePose3, zonePose4, zonePose5, zonePose6, zonePose7;
    public Pose[] zonePoses;


    public static double parkX = 59.64, parkY = 14.73, parkHeading = Math.PI/2;
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
        zonePose3 = new Pose(zone3X, zone3Y * m, toBlobHeading(zone3Heading * m));
        zonePose4 = new Pose(zone4X, zone4Y * m, toBlobHeading(zone4Heading * m));
        zonePose5 = new Pose(zone5X, zone5Y * m, toBlobHeading(zone5Heading * m));
        zonePose6 = new Pose(zone6X, zone6Y * m, toBlobHeading(zone6Heading * m));
        zonePose7 = new Pose(zone7X, zone7Y * m, toBlobHeading(zone7Heading * m));
        zonePoses = new Pose[]{zonePose0, zonePose1, zonePose2, zonePose3,
                zonePose4, zonePose5, zonePose6, zonePose7};

        parkPose = new Pose(parkX, parkY * m, toBlobHeading(parkHeading * m));
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
    public double getHpIntakeUntilPercentage() { return hpIntakeUntilPercentage; }
    public double getFailSafeDtTime() { return failSafeDtTime; }
    public double getScoreFailSafeDtTime() { return scoreFailSafeDtTime; }
    public double getScoreSettleDelay() { return scoreSettleDelay; }
    public long getFailSafePickupTime() { return failSafePickupTime; }
    public double getZoneDecideTimeout() { return zoneDecideTimeout; }
    public int getCycleCount() { return cycleCount; }
}
