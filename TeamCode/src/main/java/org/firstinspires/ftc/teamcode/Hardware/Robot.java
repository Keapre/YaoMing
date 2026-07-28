package org.firstinspires.ftc.teamcode.Hardware;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.Hardware.Intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Launcher;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.TeleOP;
import org.firstinspires.ftc.teamcode.Util.Globals.Phase;
import org.firstinspires.ftc.teamcode.Util.Info;
import org.firstinspires.ftc.teamcode.blob.driveTrain.Blob;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Intake.IntakeTransfer;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Turret;
import org.firstinspires.ftc.teamcode.Hardware.Vision.Limelight;
import org.firstinspires.ftc.teamcode.Util.Wrapper.TelemetryUtil;
import org.firstinspires.ftc.teamcode.Util.Controllers.velocityController;

import java.util.List;

@Config
public class Robot {
    public HardwareMap hw;
    public IntakeTransfer intakeTransfer;
    public Outtake outtake;
    public Sensors sensors;
    public Blob blob = null;
    public Limelight limelight;
    public OpMode op;
    public static boolean showTelemetry = false;
    Telemetry telemetry;

    // --- Profiling Variables ---
    private long lastLoopTime = 0;
    private double loopTimeMs = 0;
    private double blobTimeMs = 0;
    private double sensorsTimeMs = 0;
    private double intakeTimeMs = 0;
    private double outtakeTimeMs = 0;

    long currentLoopTime;
    long t1,t2,t3,t4,t5;

    LynxModule cHub;

    public Robot(OpMode op)
    {
        cHub = op.hardwareMap.get(LynxModule.class, "Control Hub");
        cHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        this.op = op;
        this.hw = op.hardwareMap;
        blob = new Blob(op.hardwareMap, Blob.State.PID);
        sensors = new Sensors(this);
        intakeTransfer = new IntakeTransfer(this,sensors);
        outtake = new Outtake(this,sensors);
        limelight = new Limelight(this);
        TelemetryUtil.setup();
    }



    public void update() {
        // Measure total loop time
        currentLoopTime = System.nanoTime();
        if (lastLoopTime != 0) {
            loopTimeMs = (currentLoopTime - lastLoopTime) / 1_000_000.0;
        }
        lastLoopTime = currentLoopTime;

        cHub.clearBulkCache();

        // Profile Blob
        t1 = System.nanoTime();
        blob.update();

        // Profile Sensors
        t2 = System.nanoTime();
        blobTimeMs = (t2 - t1) / 1_000_000.0;
        sensors.update();

        // Profile Intake
        t3 = System.nanoTime();
        sensorsTimeMs = (t3 - t2) / 1_000_000.0;
        intakeTransfer.update();

        // Profile Outtake
        t4 = System.nanoTime();
        intakeTimeMs = (t4 - t3) / 1_000_000.0;
        outtake.update();

        t5 = System.nanoTime();
        outtakeTimeMs = (t5 - t4) / 1_000_000.0;

        limelight.update();

        Info.lastPoseX = sensors.getX();
        Info.lastPoseY = sensors.getY();
        Info.lastPoseHeading = sensors.getHeading();
        if (!Info.hasLastPose) Info.hasLastPose = true;

        if(showTelemetry) {
            updateTelemetry();
            TelemetryUtil.sendTelemetry();
        }
    }

    // Returning loop time in milliseconds is generally more readable,
    // but I kept the method signature the same just in case you use it elsewhere.
    public long getLoopTimeNs() {
        return (long) (loopTimeMs * 1_000_000.0);
    }

    public void updateTelemetry() {

        TelemetryUtil.packet.addLine("--- Loop Profiling (ms) ---");
        TelemetryUtil.packet.put("Total Loop Time", loopTimeMs);
        TelemetryUtil.packet.put("Blob Update", blobTimeMs);
        TelemetryUtil.packet.put("Sensors Update", sensorsTimeMs);
        TelemetryUtil.packet.put("Intake Update", intakeTimeMs);
        TelemetryUtil.packet.put("Outtake Update", outtakeTimeMs);

        TelemetryUtil.packet.addLine("--- Robot Telemetry ---");
        TelemetryUtil.packet.put("Intake State", intakeTransfer.intakeState);
        TelemetryUtil.packet.put("Previous State", intakeTransfer.previousState);
        TelemetryUtil.packet.put("Outtake State", outtake.outtakeState);
        TelemetryUtil.packet.put("Launcher State", outtake.launcher.launcherState);
        TelemetryUtil.packet.put("Turret State", outtake.turret.turretState);

        TelemetryUtil.packet.put("Launcher Target ", outtake.launcher.target);
        TelemetryUtil.packet.put("Launcher Velocity ", outtake.launcher.currentVel);

        TelemetryUtil.packet.put("Intake amps",intakeTransfer.intake.getCurrent(CurrentUnit.AMPS));
        TelemetryUtil.packet.put("Shooter1 amps", outtake.launcher.getMotor1Amps());
        TelemetryUtil.packet.put("Shooter2 amps", outtake.launcher.getMotor2Amps());
        TelemetryUtil.packet.put("Shooter1 mAh", outtake.launcher.getMotor1mAh());
        TelemetryUtil.packet.put("Shooter2 mAh", outtake.launcher.getMotor2mAh());

        TelemetryUtil.packet.put("Current X", sensors.getX());
        TelemetryUtil.packet.put("Current Y", sensors.getY());
        TelemetryUtil.packet.put("Current heading", sensors.getHeading());
        TelemetryUtil.packet.put("voltage",sensors.getVoltage());
        TelemetryUtil.packet.put("beam 1 value",sensors.isBreakBeamPos1Low());
        TelemetryUtil.packet.put("beam 2 value",sensors.isBreakBeamPos2Low());
        TelemetryUtil.packet.put("beam 3 value",sensors.isBreakBeamPos3Low());
        TelemetryUtil.packet.put("beam 3 first update",sensors.firstTrueBeam3);
        TelemetryUtil.packet.put("Sensor light",sensors.lightColor.toString());
        TelemetryUtil.packet.put("Hood Target", outtake.launcher.getHoodPosition());

        Canvas field = TelemetryUtil.getPacket().fieldOverlay();
        double rxIn = sensors.getX();
        double ryIn = sensors.getY();
        double rh = sensors.getHeading(); // radians
        double headingVecLenIn = 7; // inch
        field.setStroke("#00ccff");
        field.strokeCircle(rxIn, ryIn, 7);
        field.strokeLine(rxIn, ryIn, rxIn + Math.cos(rh) * headingVecLenIn, ryIn + Math.sin(rh) * headingVecLenIn);
        double shootTx = sensors.getTargetX();
        double shootTy = sensors.getTargetY();
        double movingGoalX = sensors.getMoveGoalX();
        double movingGoalY = sensors.getMoveGoalY();
        double shooterX = sensors.getShooterX();
        double shooterY = sensors.getShooterY();

        field.setStroke("#ff3366");
        field.strokeCircle(shootTx, shootTy, 2);

        field.setStroke("#aa33ff");
        field.strokeLine(rxIn, ryIn, shootTx, shootTy);
        field.setStroke("#27f576");
        field.strokeCircle(shooterX, shooterY, 2);
        field.setStroke("#B33D29");
        field.strokeCircle(sensors.projectedX, sensors.projectedY, 2);
        field.setStroke("#4ef542");
        field.strokeCircle(movingGoalX,movingGoalY,2);

        // pidPredict close-zone triangle + robot projection line (when projecting)
        double[] zoneX = {velocityController.PRED_CLOSEZONE_X1, velocityController.PRED_CLOSEZONE_X2, velocityController.PRED_CLOSEZONE_X3};
        double[] zoneY = {velocityController.PRED_CLOSEZONE_Y1, velocityController.PRED_CLOSEZONE_Y2, velocityController.PRED_CLOSEZONE_Y3};
        field.setStroke("#ffaa00");
        field.strokePolygon(zoneX, zoneY);
        if (outtake.launcher.predicting) {
            field.strokeCircle(outtake.launcher.predictX, outtake.launcher.predictY, 3);
            field.strokeLine(shooterX, shooterY, outtake.launcher.predictX, outtake.launcher.predictY);
        }

        TelemetryUtil.packet.put("Robot velX", sensors.getVelX());
        TelemetryUtil.packet.put("Robot velY", sensors.getVelY());
    }
}