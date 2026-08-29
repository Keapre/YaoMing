package org.firstinspires.ftc.teamcode.OpMode.Auto.Far.Far;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.blob.Blob;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Intake.IntakeTransfer;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Launcher;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Turret;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Util.Globals.Phase;
import org.firstinspires.ftc.teamcode.Util.Info;
import org.firstinspires.ftc.teamcode.Util.Wrapper.TelemetryUtil;

public class Far extends OpMode {
    Robot robot;
    FarConstants constants;

    Timer pathTimer;
    ElapsedTime timerAuto = null;

    int cycleCounter = 0;
    int chosenZone = -1;
    Pose chosenZonePose = null;
    int selectedPipeline = FarConstants.limelightPipeline;

    // Latched side-veer for the current pickup (decided once, then held, to avoid pump-faking).
    boolean veerDecided = false;
    double veerOx = 0, veerOy = 0;
    int veerSide = 0; // +1 = veering right, -1 = left, 0 = none

    public enum AutoStates {
        IDLE,
        SHOOT_PRELOAD,
        GO_PICKUP_HP,
        WAIT_PICKUP_HP,
        GO_SCORE_HP,
        WAIT_SCORE_HP,
        GO_PICKUP3_INTER,
        GO_PICKUP3,
        WAIT_PICKUP3,
        GO_SCORE_SPIKE3,
        WAIT_SCORE_SPIKE3,
        DECIDE_ZONE,
        GO_ZONE_PICKUP,
        PICKUP_FAILSAFE,
        WAIT_PICKUP_FAILSAFE,
        WAIT_ZONE_PICKUP,
        GO_SCORE_CYCLE,
        WAIT_SCORE_CYCLE,
        GO_TO_PARK,
        PARK,
        SLEEP
    }

    public AutoStates autoStates = AutoStates.IDLE;
    public AutoStates prevAutoStates = AutoStates.IDLE;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Info.phase = Phase.AUTONOMOUS;
        Info.useBlob = true;
        robot = new Robot(this);

        constants = new FarConstants();
        constants.buildPaths();

        robot.blob.setMode(FarConstants.useRST ? Blob.State.RST : Blob.State.PID);

        robot.outtake.launcher.closeMode = false;
        robot.limelight.pipelineSwitch(FarConstants.limelightPipeline);
        robot.limelight.setLamp(true);
        robot.outtake.launcher.autoAimOn(true);
        robot.outtake.outtakeState = Outtake.OuttakeState.IDLE;
        robot.sensors.setPoseAlign(false);
        robot.outtake.turret.turretState = Turret.TurretState.TRACKING;

    }

    @Override
    public void init_loop() {
        robot.blob.setPose(constants.startPose);
        robot.blob.odo.update();
        robot.sensors.update();
        robot.outtake.primeAimForAuto();

        telemetry.addData("Limelight pipeline", selectedPipeline);
        telemetry.update();
    }

    @Override
    public void start() {
        robot.blob.setPose(constants.startPose);
        robot.blob.odo.update();
        double startHeading = robot.blob.odo.getHeading();
        robot.blob.turnToRadians((startHeading < 0) ? Math.abs(startHeading) : (2 * Math.PI - startHeading));
        pathTimer = new Timer();
        cycleCounter = 0;
        timerAuto = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timerAuto.startTime();
        setPathState(AutoStates.SHOOT_PRELOAD);
    }

    @Override
    public void loop() {
        telemetry.addData("auto state", autoStates);
        telemetry.addData("cycle", cycleCounter + "/" + constants.getCycleCount());
        telemetry.addData("LL zone counts", java.util.Arrays.toString(robot.limelight.getZoneCounts()));
        telemetry.addData("LL target zone", robot.limelight.getZone());
        telemetry.addData("LL hasTarget", robot.limelight.hasTarget());
        telemetry.addData("Drive inPos", robot.blob.inPosition());
        telemetry.addData("outtake state", robot.outtake.outtakeState);
        telemetry.addData("veerDecided", veerDecided);
        telemetry.addData("dist to target", robot.sensors.getDistanceToBackboard());
        telemetry.addData("shooter vel", robot.outtake.launcher.currentVel);
        telemetry.addData("shooter target vel", robot.outtake.launcher.target);
        telemetry.update();

        holdLiveAim();

        switch (autoStates) {
            case IDLE:
                break;

            case SHOOT_PRELOAD:
                robot.blob.setTargetPosition(constants.startPose.getX(), constants.startPose.getY());
                robot.outtake.turret.turretState = Turret.TurretState.TRACKING;

                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF);
                if (!robot.outtake.launcher.isReady() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                robot.outtake.start_feed_rapid(constants.getLauncherVelocity(), constants.getHoodPosition());
                sleep(constants.getShootingTime(), AutoStates.GO_PICKUP3_INTER, true);
                break;

            case GO_PICKUP_HP:
                robot.blob.setTargetPosition(constants.humanPlayerPose);
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.INTAKE);
                if (!robot.blob.inPosition(1.6,1.6,0.12) && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                setPathState(AutoStates.WAIT_PICKUP_HP);
                break;
            case WAIT_PICKUP_HP:
                if (robot.sensors.areAllLasersBlockedForTime() || pathTimer.getElapsedTime() > constants.getFailSafePickupTime()) {
                    setPathState(AutoStates.GO_SCORE_HP);
                }
                break;

            case GO_SCORE_HP:
                robot.blob.setTargetPosition(constants.scorePose);
                setPathState(AutoStates.WAIT_SCORE_HP);
                break;
            case WAIT_SCORE_HP:
                if (pathFraction(constants.humanPlayerPose, constants.scorePose) < constants.getHpIntakeUntilPercentage()) {
                    robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.INTAKE);
                } else {
                    robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF_OPEN);
                }
                if (!robot.blob.inPosition(1.6,1.6,0.12)) break;
                robot.outtake.start_feed_rapid(constants.getLauncherVelocity(), constants.getHoodPosition());
                sleep(constants.getShootingTime(), AutoStates.DECIDE_ZONE, true);
                break;

            case GO_PICKUP3_INTER:
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF);
                robot.blob.setTargetPosition(constants.pickUpPose3Intermediary);
                if (!robot.blob.inPosition(1.6,1.6,0.12) && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                setPathState(AutoStates.GO_PICKUP3);
                break;
            case GO_PICKUP3:
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.INTAKE);
                robot.blob.setTargetPosition(constants.pickUpPose3);
                if (!robot.blob.inPosition(1.6,1.6,0.12) && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                setPathState(AutoStates.WAIT_PICKUP3);
                break;
            case WAIT_PICKUP3:
                if (robot.sensors.areAllLasersBlockedForTime() || pathTimer.getElapsedTime() > constants.getFailSafePickupTime()) {
                    setPathState(AutoStates.GO_SCORE_SPIKE3);
                }
                break;
            case GO_SCORE_SPIKE3:
                robot.blob.setTargetPosition(constants.scorePose);
                setPathState(AutoStates.WAIT_SCORE_SPIKE3);
                break;
            case WAIT_SCORE_SPIKE3:
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF_OPEN);
                if (!robot.blob.inPosition(1.6,1.6,0.12)) break;
                robot.outtake.start_feed_rapid(constants.getLauncherVelocity(), constants.getHoodPosition());
                sleep(constants.getShootingTime(), AutoStates.GO_PICKUP_HP, true);
                break;

            case DECIDE_ZONE:
                chosenZone = robot.limelight.getZone();
                if (chosenZone < 0 && pathTimer.getElapsedTime() < constants.getZoneDecideTimeout()) break;
                int poseIdx = constants.poseIndexFor(chosenZone);
                if (chosenZone >= 0 && robot.limelight.getAvgZoneSpeed() > constants.getZoneSpeedThreshold()) {
                    poseIdx = Math.max(0, Math.min(constants.zonePoses.length - 1,
                            poseIdx + FarConstants.zoneSpeedShiftPose));
                }
                chosenZonePose = constants.zonePoses[poseIdx];
                veerDecided = false; veerOx = 0; veerOy = 0; veerSide = 0; // fresh veer decision for this pickup
                setPathState(AutoStates.GO_ZONE_PICKUP);
                break;
            case GO_ZONE_PICKUP:
                commandZonePickupDrive();
                if (!robot.blob.inPosition(1.6,1.6,0.12) && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                setPathState(AutoStates.WAIT_ZONE_PICKUP);
                break;
            case WAIT_ZONE_PICKUP:
                if (robot.sensors.areAllLasersBlockedForTime() || pathTimer.getElapsedTime() > constants.getFailSafePickupTime()) {
                    setPathState(AutoStates.GO_SCORE_CYCLE);
                }
                break;
            case GO_SCORE_CYCLE:
                robot.blob.maxPower = 1.0;
                robot.blob.setTargetPosition(constants.scorePose);
                robot.outtake.turret.turretState = Turret.TurretState.TRACKING;
                setPathState(AutoStates.WAIT_SCORE_CYCLE);
                break;
            case WAIT_SCORE_CYCLE:
                if (pathFraction(chosenZonePose, constants.scorePose) < constants.getCycleIntakeUntilPercentage()) {
                    robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.INTAKE);
                } else {
                    robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF_OPEN);
                }
                if (!robot.blob.inPosition(1.6,1.6,0.12)) break;
                robot.outtake.start_feed_rapid(constants.getLauncherVelocity(), constants.getHoodPosition());
                cycleCounter++;
                if (cycleCounter < constants.getCycleCount()) {
                    sleep(constants.getShootingTime(), AutoStates.DECIDE_ZONE, true);
                } else {
                    sleep(constants.getShootingTime(), AutoStates.GO_TO_PARK, true);
                }
                break;

            // --- 6. Park ---
            case GO_TO_PARK:
                robot.outtake.setOuttakeState(Outtake.OuttakeState.IDLE);
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF);
                robot.blob.setTargetPosition(constants.parkPose);
                setPathState(AutoStates.PARK);
                break;
            case PARK:
                if (!robot.blob.inPosition(1.6,1.6,0.12)) break;
                if (robot != null) robot.limelight.setLamp(false);
                requestOpModeStop();
                break;

            case SLEEP:
                if (!entered) {
                    if (robot.outtake.launcher.launcherState == Launcher.LauncherState.LAUNCHING) {
                        startSleep = System.currentTimeMillis();
                        entered = true;
                    }
                } else if (System.currentTimeMillis() - startSleep > sleeptime) {
                    setPathState(nextState);
                }
                break;
        }

        // The follower refuses to drive on constants that cannot be right, which in a match would
        // be a stationary auto. Fall back rather than stop, and say so loudly.
        if (robot.blob.rstFault != null) {
            telemetry.addData("RST FAULT, using PID", robot.blob.rstFault);
            robot.blob.setMode(Blob.State.PID);
        } else {
            // Live toggle, so it can be dropped back to the PID from the dashboard mid-match.
            robot.blob.setMode(FarConstants.useRST ? Blob.State.RST : Blob.State.PID);
            if (FarConstants.useRST) {
                telemetry.addData("RST v ref", Math.hypot(
                        robot.blob.rst.forwardReference, robot.blob.rst.lateralReference));
                telemetry.addData("RST v now", robot.blob.odo.getSpeedTranslational());
            }
        }

        TelemetryUtil.packet.put("x", robot.blob.odo.getX());
        TelemetryUtil.packet.put("y", robot.blob.odo.getY());
        TelemetryUtil.packet.put("heading", robot.blob.odo.getHeading());
        TelemetryUtil.sendTelemetry();
        prevAutoStates = autoStates;
        robot.update();
    }

    /** Flushes the blob path trace to disk. Without this the final segment never lands. */
    @Override
    public void stop() {
        if (robot != null) robot.blob.saveTrace();
    }

/**
     * Live aim: parks the outtake in IDLE, whose adaptive branch re-solves vel + hood from the live
     * odometry distance every loop (and keeps the wheel spinning at that target rather than winding
     * down between cycles). Skipped during SLEEP, where the feed state machine owns the outtake.
     */
    private void holdLiveAim() {
        if (autoStates == AutoStates.SLEEP) return;
        robot.outtake.setOuttakeState(Outtake.OuttakeState.IDLE);
    }

    long startSleep = 0;
    double sleeptime = 0;
    AutoStates nextState = AutoStates.IDLE;
    boolean entered = false;

    private void sleep(double time, AutoStates nextState, boolean shooting) {
        entered = !shooting;
        startSleep = System.currentTimeMillis();
        setPathState(AutoStates.SLEEP);
        sleeptime = time;
        this.nextState = nextState;
    }

    public void setPathState(AutoStates pState) {
        autoStates = pState;
        pathTimer.resetTimer();
        scoreReachedTime = 0;
    }

    private long scoreReachedTime = 0;

    private boolean readyToShoot(boolean inPos) {
        if (pathTimer.getElapsedTime() >= constants.getScoreFailSafeDtTime()) return true;
        if (!inPos) {
            scoreReachedTime = 0;
            return false;
        }
        if (scoreReachedTime == 0) scoreReachedTime = System.currentTimeMillis();
        return System.currentTimeMillis() - scoreReachedTime >= constants.getScoreSettleDelay();
    }

    private double pathFraction(Pose start, Pose end) {
        if (start == null || end == null) return 1.0;
        double dx = end.getX() - start.getX();
        double dy = end.getY() - start.getY();
        double lenSq = dx * dx + dy * dy;
        if (lenSq < 1e-6) return 1.0;
        double px = robot.blob.odo.getX() - start.getX();
        double py = robot.blob.odo.getY() - start.getY();
        double t = (px * dx + py * dy) / lenSq;
        return Math.max(0.0, Math.min(1.0, t));
    }
    
    private void commandZonePickupDrive() {
        robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.INTAKE);

        robot.blob.maxPower = 1.0;
        double frac = pathFraction(constants.scorePose, chosenZonePose);

        if (!veerDecided && frac >= constants.getRescanPercentage() && robot.limelight.hasTarget()) {
            int lft = robot.limelight.getLeftCount();
            int ctr = robot.limelight.getCenterCount();
            int rgt = robot.limelight.getRightCount();
            double h = robot.blob.odo.getHeading();
            double rX = Math.sin(h), rY = -Math.cos(h); // robot's right in field coords
            double sRight = constants.getSideShiftRightInches();
            double sLeft = constants.getSideShiftLeftInches();
            if (rgt > ctr && rgt >= lft) { veerOx = rX * sRight; veerOy = rY * sRight; veerSide = 1; veerDecided = true; }
            else if (lft > ctr && lft > rgt) { veerOx = -rX * sLeft; veerOy = -rY * sLeft; veerSide = -1; veerDecided = true; }
        }
        double baseX = (veerSide > 0) ? robot.blob.odo.getX() : chosenZonePose.getX();
        robot.blob.setTargetPosition(new Pose(baseX + veerOx, chosenZonePose.getY() + veerOy,
                chosenZonePose.getHeading()));
    }
}
