package org.firstinspires.ftc.teamcode.OpMode.Auto.BallChase;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.Intake.IntakeTransfer;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Util.Wrapper.GamePadController;
import org.firstinspires.ftc.teamcode.blob.driveTrain.Blob;

@Config
@Autonomous(name = "Limelight Ball Chase")
public class LimelightBallChaseAuto extends LinearOpMode {

    public static int PIPELINE = 9;
    public static double MAX_FORWARD_INCHES = 78.0;

    public static double DRIVE_SPEED = 0.65;
    public static double K_STEER = 0.02;       // rotation power per degree of tx
    public static double MAX_TURN = 0.2;      // clamp on rotation power
    public static double TX_SLOW_DEG = 20;   // above this |tx|, forward tapers to 0 (turn in place first)

    private Robot robot;
    private GamePadController gg;

    enum Phase { IDLE, CHASING }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        gg = new GamePadController(gamepad1);

        robot = new Robot(this);
        robot.blob.setMode(Blob.State.DRIVE);
        robot.outtake.outtakeState = Outtake.OuttakeState.IDLE;
        robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF);
        robot.limelight.pipelineSwitch(PIPELINE);

        telemetry.update();

        waitForStart();

        Phase phase = Phase.IDLE;
        double distTraveled = 0.0;
        double prevX = 0.0, prevY = 0.0;

        while (opModeIsActive()) {
            gg.update();
            robot.update();

            double tx = robot.limelight.getTx();
            double ballCount = robot.limelight.getBallCount();
            boolean hasTarget = robot.limelight.hasTarget();

            if (gg.bOnce()) {
                break;
            }

            if (phase == Phase.IDLE) {
                robot.blob.setTargetVector(0, 0, 0);
                if (gg.aOnce()) {
                    phase = Phase.CHASING;
                    distTraveled = 0.0;
                    prevX = robot.blob.odo.getX();
                    prevY = robot.blob.odo.getY();
                    robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.INTAKE);
                }
            } else {
                double nx = robot.blob.odo.getX();
                double ny = robot.blob.odo.getY();
                distTraveled += Math.hypot(nx - prevX, ny - prevY);
                prevX = nx;
                prevY = ny;

                if (distTraveled >= MAX_FORWARD_INCHES) {
                    break;
                }

                if (hasTarget) {
                    double rotate = Range.clip(K_STEER * tx, -MAX_TURN, MAX_TURN);
                    double slow = Range.clip(Math.abs(tx) / TX_SLOW_DEG, 0.0, 1.0);
                    double forward = DRIVE_SPEED * (1.0 - slow);
                    robot.blob.setTargetVector(0, forward, rotate);
                } else {
                    robot.blob.setTargetVector(0, 0, 0);
                }
            }

            telemetry.addData("phase", phase);
            telemetry.addData("hasTarget", hasTarget);
            telemetry.addData("tx (deg)", tx);
            telemetry.addData("ballCount", ballCount);
            telemetry.addData("distTraveled (in)", distTraveled);
            telemetry.update();
        }

        robot.blob.setTargetVector(0, 0, 0);
        robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF);
    }
}
