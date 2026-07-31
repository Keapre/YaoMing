package org.firstinspires.ftc.teamcode.OpMode.Auto.Far.Far;

import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
        Info.phase = Phase.AUTONOMOUS;
        Info.useBlob = true;
        robot = new Robot(this);

        constants = new FarConstants();
        constants.buildPaths();

        robot.outtake.launcher.closeMode = false;
        robot.limelight.pipelineSwitch(FarConstants.limelightPipeline);
        robot.limelight.setLamp(true);
        robot.outtake.launcher.autoAimOn(false);
        robot.outtake.outtakeState = Outtake.OuttakeState.IDLE;
        robot.sensors.setPoseAlign(false);
        robot.outtake.turret.turretState = Turret.TurretState.TRACKING;

    }

    @Override
    public void init_loop() {
        robot.blob.odo.setPose(constants.startPose);
        robot.blob.odo.update();
        robot.sensors.update();
        robot.outtake.primeAimForAuto();

        if (gamepad1.a && selectedPipeline != 9) {
            selectedPipeline = 9;
            robot.limelight.pipelineSwitch(selectedPipeline);
        } else if (gamepad1.b && selectedPipeline != 8) {
            selectedPipeline = 8;
            robot.limelight.pipelineSwitch(selectedPipeline);
        }

        telemetry.addLine("Limelight pipeline select:");
        telemetry.addData("  A", "pipeline 9  (RED LIGHT SHINING ON THE ROBOT)");
        telemetry.addData("  B", "pipeline 8  (BLUE LIGHT SHINING ON THE ROBOT)");
        telemetry.addData("Selected pipeline",
                selectedPipeline + (selectedPipeline == 9 ? "  (RED light)" : "  (BLUE light)"));
        telemetry.update();
    }

    @Override
    public void start() {
        robot.blob.odo.setPose(constants.startPose);
        robot.blob.odo.update();
        double startHeading = robot.blob.odo.getHeading();
        robot.blob.targetHeading = (startHeading < 0) ? Math.abs(startHeading) : (2 * Math.PI - startHeading);
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
        telemetry.addData("LL zone", robot.limelight.getZone());
        telemetry.addData("LL hasTarget", robot.limelight.hasTarget());
        telemetry.addData("Drive inPos", robot.blob.inPosition());
        telemetry.addData("outtake state", robot.outtake.outtakeState);
        telemetry.update();

        switch (autoStates) {
            case IDLE:
                break;

            case SHOOT_PRELOAD:
                robot.blob.setTargetPosition(constants.startPose.getX(), constants.startPose.getY());
                robot.outtake.turret.turretState = Turret.TurretState.TRACKING;

                robot.outtake.specificValues(constants.startPose);
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF);
                if (!robot.outtake.launcher.isReady() && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                robot.outtake.start_feed_rapid(constants.getLauncherVelocity(), constants.getHoodPosition());
                sleep(constants.getShootingTime(), AutoStates.GO_PICKUP3_INTER, true);
                break;

            case GO_PICKUP_HP:
                robot.outtake.specificValues(constants.scorePose);
                robot.blob.setTargetPosition(constants.humanPlayerPose);
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.INTAKE);
                if (!robot.blob.inPosition(1.6,1.6,0.12) && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                setPathState(AutoStates.WAIT_PICKUP_HP);
                break;
            case WAIT_PICKUP_HP:
                if (robot.sensors.areAllBeamsLowForTime() || pathTimer.getElapsedTime() > constants.getFailSafePickupTime()) {
                    setPathState(AutoStates.GO_SCORE_HP);
                }
                break;

            case GO_SCORE_HP:
                robot.blob.setTargetPosition(constants.scorePose);
                robot.outtake.specificValues(constants.scorePose);
                setPathState(AutoStates.WAIT_SCORE_HP);
                break;
            case WAIT_SCORE_HP:
                robot.outtake.specificValues(constants.scorePose);
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
                robot.outtake.specificValues(constants.scorePose);
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
                if (robot.sensors.areAllBeamsLowForTime() || pathTimer.getElapsedTime() > constants.getFailSafePickupTime()) {
                    setPathState(AutoStates.GO_SCORE_SPIKE3);
                }
                break;
            case GO_SCORE_SPIKE3:
                robot.blob.setTargetPosition(constants.scorePose);
                robot.outtake.specificValues(constants.scorePose);
                setPathState(AutoStates.WAIT_SCORE_SPIKE3);
                break;
            case WAIT_SCORE_SPIKE3:
                robot.outtake.specificValues(constants.scorePose);
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
                setPathState(AutoStates.GO_ZONE_PICKUP);
                break;
            case GO_ZONE_PICKUP:
                robot.outtake.specificValues(constants.scorePose);
                robot.blob.setTargetPosition(chosenZonePose);
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.INTAKE);
                if (!robot.blob.inPosition(1.6,1.6,0.12) && pathTimer.getElapsedTime() < constants.getFailSafeDtTime()) break;
                setPathState(AutoStates.WAIT_ZONE_PICKUP);
                break;
            case WAIT_ZONE_PICKUP:
                if (robot.sensors.areAllBeamsLowForTime() || pathTimer.getElapsedTime() > constants.getFailSafePickupTime()) {
                    setPathState(AutoStates.GO_SCORE_CYCLE);
                }
                break;
            case GO_SCORE_CYCLE:
                robot.blob.setTargetPosition(constants.scorePose);
                robot.outtake.turret.turretState = Turret.TurretState.TRACKING;
                setPathState(AutoStates.WAIT_SCORE_CYCLE);
                break;
            case WAIT_SCORE_CYCLE:
                robot.outtake.specificValues(constants.scorePose);
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

        TelemetryUtil.packet.put("x", robot.blob.odo.x);
        TelemetryUtil.packet.put("y", robot.blob.odo.y);
        TelemetryUtil.packet.put("heading", robot.blob.odo.heading);
        TelemetryUtil.sendTelemetry();
        prevAutoStates = autoStates;
        robot.update();
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
}
