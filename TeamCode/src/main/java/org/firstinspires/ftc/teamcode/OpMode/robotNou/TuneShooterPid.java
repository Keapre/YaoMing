package org.firstinspires.ftc.teamcode.OpMode.robotNou;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Intake.IntakeTransfer;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Launcher;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.OuttakePositions;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Util.Globals.Phase;
import org.firstinspires.ftc.teamcode.Util.Info;
import org.firstinspires.ftc.teamcode.Util.Wrapper.GamePadController;
import com.blob.Blob;

@Config
@TeleOp
public class TuneShooterPid extends LinearOpMode {
    GamePadController gg;
    Robot robot;
    Pose resetCenter = new Pose(0, 0, Math.PI/2);
    @Override
    public void runOpMode() throws InterruptedException {
        gg = new GamePadController(gamepad1);
        Info.phase = Phase.TELEOP;
        robot = new Robot(this);
        waitForStart();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF);
        robot.outtake.setOuttakeState(Outtake.OuttakeState.OFF);
        robot.blob.setMode(Blob.State.DRIVE);
        while (opModeIsActive()) {
            gg.update();
            robot.update();
            if(gg.rightBumper()) {
                if(gg.rightBumperOnce()){
                    switch (robot.intakeTransfer.intakeState){
                        case INTAKE:
                        case REVERSE:
                            robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF);
                            break;
                        case OFF:
                            robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.INTAKE);
                            break;
                    }
                }
            }
            if (gg.dpadDown()) {
                if(gg.leftTrigger() && gg.rightTrigger()){
                    robot.blob.setPose(resetCenter);
                }
            }
            Outtake.OuttakeState state = robot.outtake.outtakeState;

            if (gg.aOnce() || gg.leftBumperOnce() ) { //&& robot.intakeTransfer.intakeState != IntakeTransfer.IntakeState.INTAKE
                if(robot.intakeTransfer.isRecycle()) {
                    robot.intakeTransfer.spinUpRecycle();
                }else {
                    if(state == Outtake.OuttakeState.IDLE || state == Outtake.OuttakeState.OFF){

                    robot.outtake.start_feed_rapid(OuttakePositions.farLaunchVelocity, OuttakePositions.farLaunchTilt);

                    }else {
                        // Any other non-IDLE state: STOP

                        robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF);
                        robot.outtake.setOuttakeState(Outtake.OuttakeState.OFF);
                    }
                }
            }
            robot.outtake.launcher.launcherState = Launcher.LauncherState.TUNE_PID;
            telemetry.addData("Target Velocity", robot.outtake.launcher.getTunePidTarget());
            telemetry.addData("Current Velocity", robot.outtake.launcher.currentVel);
            telemetry.addData("power",robot.outtake.launcher.getPower());
            telemetry.addData("intake state",robot.intakeTransfer.intakeState);
            telemetry.addData("launcher state",robot.outtake.launcher.launcherState);
            telemetry.addData("distance",robot.sensors.getDistanceToBackboard());
            telemetry.addData("heading", robot.sensors.getHeading());
            telemetry.addData("heading degrees", Math.toDegrees(robot.sensors.getHeading()));
            telemetry.addData("x", robot.blob.odo.getPose().getX());
            telemetry.addData("y", robot.blob.odo.getPose().getY());
            telemetry.update();
        }
    }
}
