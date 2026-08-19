package org.firstinspires.ftc.teamcode.OpMode.Auto.nevagonause.Close.Close18Playoff;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Util.Globals.Alliance;
import org.firstinspires.ftc.teamcode.Util.Info;

@Config
public class CloseConstants30 {

    //DEFAULT VALUES FOR RED
    public static double turretTargetPos = 0.364;
    public static double shootingTime = 250;
    public static double shootingTimeSOTM = 150;
    public static double shootingTimeSOTMdelay = 100;
    public static double preloadWait = 100;
    public static double shootingPercentage = 0.8;
    public static double turretPositionRed = 0.315;

    public static double pickUp2Percentage = 0.8;
    public static double pickUp3Percentage = 0.8;
    public static double pickUp1Percentage = 0.8;
    public static double turretPositionBlue = 0.636;
    public static double failSafeDtTime = 1250;
    public static double goToGateTimer = 1000;
    public static double hoodPosition = 0.615;
    public static double preloadSotmPerc = 0.35;
    public static double launcherVelocity = 1000;
    public static double waitBeforeClear = 500;
    public static double clearTimeAfterCycle = 550;
    public static double startX = -57.14066633089322, startY = 43.2188019039124, headingStartRed = -4.0555341879474085; //TODO
    public static double startXBlue = -57.342159241203255, startYBlue = -43.25824407141979, headingStartBlue = -2.1819510618792933; //TODO
    public Pose startPose;
    public static double shootingX = -7, shootingY = 16, shootingHeading = Math.toRadians(300);
    public static double shootingS1X = -10, shootingS1Y = 22, shootingS1Heading = Math.toRadians(270);
    public static double shootPreloadX = -14.5,shootPreloadY = 16,shootPreloadH = Math.toRadians(220);
    public static double rotatePreloadX = -14.5,rotatePreloadY = 12,rotatePreloadH = Math.toRadians(270);
    public static double shootingInterX = -9, shootingInterY = 18, shootingUnterHeading = Math.toRadians(270);
    public Pose scorePose;
    public Pose scoreS1Pose;
    public Pose scorePoseGateInter;

    public static double pickUp1X = -13, pickUp1Y = 50.3, pickUp1Heading = Math.toRadians(270);
    public static double InterpickUp1X = -13, InterpickUp1Y = 22, InterpickUp1Heading = Math.toRadians(270);
    public Pose pickUpPose;
    public Pose pickUpPose1_Inter;
    public static double max_power_pickUp = 1;
    public static double pickUp2X = 10.9,pickUp2Y = 21.5, pickUp2Heading = Math.toRadians(270);
    public static double pickUp2X2 = 10.2,pickUp2Y2 = 52, pickUp2Heading2 = Math.toRadians(270);
    public static double clearGateX = 5, clearGateY = 52, clearGateHeading = Math.toRadians(270);
    public Pose clearGateAfterCycle;
    public Pose pickUpPose2;
    public Pose pickUpPose2_2;

    public static double pickUp3X = 36,pickUp3Y = 24, pickUp3Heading = Math.toRadians(270);
    public static double pickUp3X2 = 36,pickUp3Y2 = 53, pickUp3Heading2 = Math.toRadians(270);
    public Pose pickUpPose3;
    public Pose pickUpPose3_2;
    //gate
    public static long failSafePickupTime = 1100;
    public static int gateCycleCount = 5;
    public static double turretPreloadRed = 0.49;
    public static double turretPreloadBlue = 0.455;
    public static double failSafeGateTime = 900;
    public static int gateClearCount1 = 1;
    public static int gateClearCount2 = 2;
    public static int gateClearCount3 = 3;
    public static int gateClearCount4 = 0;
    public static int headingTheeshold = 40;
    public static double gatePickupX = 12.7, gatePickupY = 55.2, gatePickupHeading = 4.34019301402063;
    public static double gatePickupXFailSafe = 14.5, gatePickupYFailSafe = 56.7, gatePickupHeadingFailSafe = 4.350019301402063;
    public Pose gatePickupPose;
    public static double percentage = 0.75;

    public static double stack1Take = 1;
    //clear gate
    public static double clearX = 0, clearY = 55.9, clearHeading = -3.1444059;
    public static double clearXInter = -5, clearYInter = 51, clearHeadingInter = 4.71;
    public Pose clearInter;

    public static double clearGateTime = 1750;


    public static double parkX = -3.5, parkY = 24.7, parkHeading = Math.toRadians(90)-Math.PI;
    public static double parkX24 = -26.55397670475517, parkY24 = 14.614846537432333, parkHeading24 = -5.501498047505514;

    public static double timerFailsafeGateLastRun = 29500;
    public static double parkFailSafe = 30000;
    public static double clearStack2Timer = 850;
    public static double timeBeforePreloadSOTM = 570;
    public static double preloadVel = 1550;
    public static double preloadHood = 0.11;
    public Pose parkPose;
    public Pose parkPose24;
    public Pose clear;
    public Pose gateFailSafe;

    public Pose preload;
    public Pose rotatePreload;

    public void buildPaths() {

        parkPose = new Pose(parkX, parkY * (Info.alliance == Alliance.RED ? 1 : -1), parkHeading * (Info.alliance == Alliance.RED ? 1 : -1));
        parkPose24 = new Pose(parkX24, parkY24 * (Info.alliance == Alliance.RED ? 1 : -1), parkHeading24 * (Info.alliance == Alliance.RED ? 1 : -1));
        if(Info.alliance == Alliance.RED){startPose = new Pose(startX, startY, headingStartRed);
        } else {startPose = new Pose(startXBlue, startYBlue, headingStartBlue);}

        scorePose = new Pose(shootingX, shootingY * (Info.alliance == Alliance.RED ? 1 : -1), shootingHeading * (Info.alliance == Alliance.RED ? 1 : -1));
        scoreS1Pose = new Pose(shootingS1X, shootingS1Y * (Info.alliance == Alliance.RED ? 1 : -1), shootingS1Heading * (Info.alliance == Alliance.RED ? 1 : -1));
        scorePoseGateInter = new Pose(shootingInterX, shootingInterY * (Info.alliance == Alliance.RED ? 1 : -1), shootingUnterHeading * (Info.alliance == Alliance.RED ? 1 : -1));
        clearInter = new Pose(clearXInter, clearYInter * (Info.alliance == Alliance.RED ? 1 : -1), clearHeadingInter * (Info.alliance == Alliance.RED ? 1 : -1));
        preload = new Pose(shootPreloadX, shootPreloadY * (Info.alliance == Alliance.RED ? 1 : -1), shootPreloadH * (Info.alliance == Alliance.RED ? 1 : -1));
        rotatePreload = new Pose(rotatePreloadX, rotatePreloadY * (Info.alliance == Alliance.RED ? 1 : -1), rotatePreloadH * (Info.alliance == Alliance.RED ? 1 : -1));
        clearGateAfterCycle = new Pose(clearGateX, clearGateY * (Info.alliance == Alliance.RED ? 1 : -1), clearGateHeading * (Info.alliance == Alliance.RED ? 1 : -1));
        pickUpPose = new Pose(pickUp1X, pickUp1Y * (Info.alliance == Alliance.RED ? 1 : -1), pickUp1Heading * (Info.alliance == Alliance.RED ? 1 : -1));
        pickUpPose1_Inter = new Pose(InterpickUp1X, InterpickUp1Y * (Info.alliance == Alliance.RED ? 1 : -1), InterpickUp1Heading * (Info.alliance == Alliance.RED ? 1 : -1));
        pickUpPose2 = new Pose(pickUp2X, pickUp2Y * (Info.alliance == Alliance.RED ? 1 : -1), pickUp2Heading * (Info.alliance == Alliance.RED ? 1 : -1));
        pickUpPose2_2 = new Pose(pickUp2X2, pickUp2Y2 * (Info.alliance == Alliance.RED ? 1 : -1), pickUp2Heading2 * (Info.alliance == Alliance.RED ? 1 : -1));
        pickUpPose3 = new Pose(pickUp3X, pickUp3Y * (Info.alliance == Alliance.RED ? 1 : -1), pickUp3Heading * (Info.alliance == Alliance.RED ? 1 : -1));
        pickUpPose3_2 = new Pose(pickUp3X2, pickUp3Y2 * (Info.alliance == Alliance.RED ? 1 : -1), pickUp3Heading2 * (Info.alliance == Alliance.RED ? 1 : -1));
        gatePickupPose = new Pose(gatePickupX, gatePickupY * (Info.alliance == Alliance.RED ? 1 : -1), gatePickupHeading * (Info.alliance == Alliance.RED ? 1 : -1));
        clear = new Pose(clearX, clearY * (Info.alliance == Alliance.RED ? 1 : -1), clearHeading * (Info.alliance == Alliance.RED ? 1 : -1));

        gateFailSafe = new Pose(gatePickupXFailSafe, gatePickupYFailSafe * (Info.alliance == Alliance.RED ? 1 : -1), gatePickupHeadingFailSafe * (Info.alliance == Alliance.RED ? 1 : -1));
//        if(Info.alliance == Alliance.BLUE) {
//            turretTargetPos = 0.5 + (0.5 - turretTargetPos);
//        }

    }

    public double getLauncherVelocity() {
        return launcherVelocity;
    }
    public double getGoToGateTimer() {
        return goToGateTimer;
    }

    public double getTurretPosition() {
        if(Info.alliance == Alliance.RED) {
            return turretPositionRed;
        }else {
            return turretPositionBlue;
        }
    }

    public double getHoodPosition() {
        return hoodPosition;
    }

    public double getShootingTime() {
        return shootingTime;
    }
    public double getShootingTimeSOTM() {
        return shootingTimeSOTM;
    }
    public double getShootingTimeSOTMdelay() {
        return shootingTimeSOTMdelay;
    }
    public double getPreloadWait() {
        return preloadWait;
    }
    public double getMaxPower(){
        return max_power_pickUp;
    }
    public double getFailSafeDtTime() {
        return failSafeDtTime;
    }
    public double getTurretTargetPos() {
        if(Info.alliance == Alliance.BLUE) {
            return turretPositionBlue;
        }else {
            return turretTargetPos;
        }
    }
    public double getFailSafePickupTime(){
        return failSafePickupTime;
    }
    public double getPreloadProgress() {
        return preloadSotmPerc;
    }
    public double getFailSafePark() {
        return parkFailSafe;
    }
    public double getTurretPosPreload() {
        if(Info.alliance == Alliance.BLUE) {
            return turretPreloadBlue;
        }else {
            return turretPreloadRed;
        }
    }
    public double getPickUp2Percentage() {
        return pickUp2Percentage;
    }
    public double getPickUp3Percentage() {
        return pickUp3Percentage;
    }
    public double getPickUp1Percentage() {
        return pickUp1Percentage;
    }
    public double getShootingPercentage() {
        return shootingPercentage;
    }
    public double getHeadingThreeshold() {
        return headingTheeshold;
    }
    public double getFailSafeGateTime() {
        return failSafeGateTime;
    }
    public double getFailSafeLastRun() {
        return timerFailsafeGateLastRun;
    }
    public double getClearGate2Time() {
        return clearStack2Timer;
    }
    public double getWaitBeforeClear() {return waitBeforeClear;}
    public double getClearTimeAfterCycle(){return clearTimeAfterCycle;}
    public double getTimeBeforePreloadSOTM(){return timeBeforePreloadSOTM;}
    public double getPreloadVel(){return preloadVel;}
    public double getPreloadHood(){return preloadHood;}
    public double getClearGateTime(){return clearGateTime;}

}
