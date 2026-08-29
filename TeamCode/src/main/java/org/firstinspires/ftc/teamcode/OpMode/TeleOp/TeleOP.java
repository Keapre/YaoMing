package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Intake.IntakeTransfer;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.OuttakePositions;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Turret;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.Sensors;
import org.firstinspires.ftc.teamcode.Util.Globals.Alliance;
import org.firstinspires.ftc.teamcode.Util.Globals.Phase;
import org.firstinspires.ftc.teamcode.Util.Info;
import org.firstinspires.ftc.teamcode.Util.Wrapper.GamePadController;
import com.blob.Blob;


@Config
public class TeleOP extends LinearOpMode
{
    public static boolean robotCentric = false;
    public static boolean ediControlls = false;
    public static boolean flipFieldFrame = false;
    public static boolean telemetryEnabled = false;
    public boolean isReadyingFlywheel = false;
    public static double driverFrameOffsetDeg = -90;
    GamePadController gg;
    GamePadController gg2;
    Robot robot;

    public static boolean isAutoAimOff = false;

    public static boolean shootWhileMoving = false;

    public static double pidTargetClosezone = 1600;
    public static double pidTargetFarzone  = 2100;

    public static boolean enableDriveSlowMode = false;
    public static boolean enableHeadingSlowMode = false;
    ElapsedTime motorTimer = new ElapsedTime();


    Pose startPose;
    Pose startPoseRed = new Pose(61.30, 31.24, Math.PI/2);
    Pose startPoseBlue = new Pose(startPoseRed.getX(),startPoseRed.getY() *-1 , - startPoseRed.getHeading());
    Pose resetPoseRed = new Pose(-8.35, 54.08, Math.PI/2); //TODO
    Pose resetPoseRedHuman = new Pose(61.48, -60.6, -Math.PI/2); //TODO
    Pose resetPoseBlue = new Pose(-8.35,-54.08, -Math.PI/2); //TODO
    Pose resetPoseBlueHuman = new Pose(61.48, 60.6 , Math.PI/2); //TODO
    Pose resetCenter = new Pose(0, 0, Math.PI/2);
    private double loopTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Info.phase = Phase.TELEOP;
        Info.useBlob = false;
        robot = new Robot(this);
        //60.48 58.22

        if (Info.hasLastPose) {
            startPose = new Pose(Info.lastPoseX, Info.lastPoseY, Info.lastPoseHeading);
        } else {
            if (Info.alliance == Alliance.RED) {
                startPose = startPoseRed;
            } else {
                startPose = startPoseBlue;
            }
        }
        gg = new GamePadController(gamepad1);
        gg2 = new GamePadController(gamepad2);


        robot.blob.setPose(startPose);
        robot.blob.odo.update();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.setMsTransmissionInterval(75);
        waitForStart();
        robot.outtake.launcher.autoAimOn(true);
        robot.sensors.setUsePredictivePose(true);
        robot.sensors.setPoseAlign(false);
        motorTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        robot.limelight.setLamp(false);

        while(opModeIsActive())
        {
            gg.update();
            gg2.update();
            updateDrive();


            robot.outtake.setShootingWhileMoving(shootWhileMoving);

            outtakeUpdate();
            intakeUpdate();

            if(telemetryEnabled){
                telemetry.addData("voltage",robot.sensors.getVoltage());
                telemetry.addData("heading",robot.blob.odo.getHeading());
                telemetry.addData("x",robot.blob.odo.getX());
                telemetry.addData("y",robot.blob.odo.getY());
                //telemetry.addData("shooterX",robot.sensors.getShooterX());
                //telemetry.addData("shooterY",robot.sensors.getShooterY());
                //telemetry.addData("dx from center",robot.sensors.getShooterX() - robot.blob.odo.getX());
                //telemetry.addData("dy from center",robot.sensors.getShooterY() - robot.blob.odo.getY());
                telemetry.addData("velocity current",robot.outtake.launcher.currentVel);
                telemetry.addData("velocity target",robot.outtake.launcher.target);
                telemetry.addData("target tilt",robot.outtake.launcher.getTarget_tilt());
                //telemetry.addData("intake State",robot.intakeTransfer.intakeState);
                //telemetry.addData("launcer State",robot.outtake.launcher.launcherState);
                telemetry.addData("velocity x",robot.sensors.getVelX());
                telemetry.addData("velocity y",robot.sensors.getVelY());
                //telemetry.addData("robot speed (in/s)",Math.hypot(robot.sensors.getVelX(), robot.sensors.getVelY()));
                telemetry.addData("power shooter",robot.outtake.launcher.getPower());
                telemetry.addData("timer", motorTimer.seconds());

                telemetry.addData("sotm",robot.sensors.sotm);
                telemetry.addData("distance to backboard", robot.sensors.getShooterDistanceToBackboard());
            }

            telemetry.addData("closeZoneMode", robot.outtake.launcher.closeMode);

            robot.update();
            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            telemetry.update();
            loopTime = loop;
        }
    }
    //posibil sa trb sa mut robot.update inainte de stall check chestie



    private double applyDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) return 0;
        return value;
    }
    public static double rotateNormal = 1;
    public static double translationalNormal = 1;
    public static double driveSlowMultiplier = 1;
    public static double headingSlowMultiplier = 1;
    public static boolean useSlowZone = false;
    double forward;
    double strafe;
    double rotate;
    double X;
    double Y;
    double rotation;

    double x;
    double allianceRotation;
    double heading ;
    double y;
    public void updateDrive() {

        if(ediControlls) {
            forward = -gg.right_stick_y;
            strafe = gg.right_stick_x;
            rotate = gg.left_stick_x;
        }
        else{
            forward = -gg.left_stick_y;
            strafe = gg.left_stick_x;
            rotate = gg.right_stick_x;
        }

        forward = applyDeadband(forward, 0.05);
        strafe = applyDeadband(strafe, 0.05);
        rotate = applyDeadband(rotate, 0.05);

        forward *= translationalNormal;
        strafe  *= rotateNormal;


        if (enableDriveSlowMode) {
            forward *= driveSlowMultiplier;
            strafe  *= driveSlowMultiplier;
        }

        if (enableHeadingSlowMode) {
            rotate *= headingSlowMultiplier;
        }

        if (robot.sensors.isInSlowZone() && useSlowZone) {
            forward *= driveSlowMultiplier;
            strafe  *= driveSlowMultiplier;
            rotate  *= headingSlowMultiplier;
        }

        robot.blob.setMode(Blob.State.DRIVE);
         X=strafe;
         Y=forward;
         rotation=rotate;

         x = X;
         y = Y;
        if (!robotCentric) {
             allianceRotation = (Info.alliance == Alliance.BLUE) ? Math.PI : 0;
             heading = -robot.blob.odo.getHeading() + Math.PI/2 + allianceRotation;

            x = X * Math.cos(heading) - Y * Math.sin(heading);
            y = X * Math.sin(heading) + Y * Math.cos(heading);
        }

        robot.blob.setTargetVector( x , y , rotation );

        if (gg.rightStickButtonOnce()) {
            Pose near = (Info.alliance == Alliance.RED) ? resetPoseRed : resetPoseBlue;
            Pose human = (Info.alliance == Alliance.RED) ? resetPoseRedHuman : resetPoseBlueHuman;
            // Right stick alone -> human player reset; both triggers held + right stick -> the other reset.
            boolean bothTriggers = gg.leftTrigger() && gg.rightTrigger();
            robot.blob.setPose(bothTriggers ? near : human);
            robot.outtake.turret.resetOffset();
            if (Info.alliance == Alliance.RED) {
                robot.outtake.launcher.manualOffset = 0;
            }
        }
        if (gg.dpadDown()) {
            if(gg.leftTrigger() && gg.rightTrigger()){
                robot.blob.setPose(resetCenter);
                robot.outtake.turret.resetOffset();
            }
        }
    }

    public void intakeUpdate() {
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
            if(gg.rightBumperLong()) {
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.REVERSE);
            }
        }
        if(!gg.rightBumper() && robot.intakeTransfer.intakeState == IntakeTransfer.IntakeState.REVERSE) {
            robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF);
        }
    }



    public static double incrementTurret = 2;
    public static double distanceIncrement = 3;
    public void outtakeUpdate() {

//        if (gg.xOnce()) {
//            shootWhileMoving = !shootWhileMoving;
//        }


        boolean isStill = true;
        if(gg.dpadRightOnce() || gg2.dpadRightOnce()) {
            robot.outtake.turret.addRemoveIncrementOffset(incrementTurret,1);
        }else if(gg.dpadLeftOnce() || gg2.dpadLeftOnce()) {
            robot.outtake.turret.addRemoveIncrementOffset(incrementTurret,-1);
        }

        if (gg.dpadUpOnce() || gg2.dpadUpOnce()) {
            robot.outtake.launcher.manualOffset = robot.outtake.launcher.manualOffset + 0.01;
        }
        if (gg.dpadDownOnce() || gg2.dpadDownOnce()) {
            robot.outtake.launcher.manualOffset = robot.outtake.launcher.manualOffset - 0.01;
        }


        if (gg.aOnce() || gg.leftBumperOnce() ) { //&& robot.intakeTransfer.intakeState != IntakeTransfer.IntakeState.INTAKE
            if(robot.intakeTransfer.isRecycle()) {
                robot.intakeTransfer.spinUpRecycle();
            }else {
                if (robot.outtake.outtakeState == Outtake.OuttakeState.IDLE) {
                    robot.outtake.start_feed_rapid(OuttakePositions.farLaunchVelocity, OuttakePositions.farLaunchTilt);
                    // Pressed shoot but flywheel not yet at velocity: keep the normal sequence, flag it on the LED.
                    if (!robot.outtake.launcher.isReady()) {
                        robot.sensors.setLedColor(Sensors.LightColor.SPINNING);
                    }

                } else {
                    // Any other non-IDLE state: STOP
                    isReadyingFlywheel = false;
                    robot.outtake.outtakeState = Outtake.OuttakeState.STOP;
                }
            }
        }
        robot.sensors.toggleSOTM(gg.left_trigger > 0.1);

//        if(gg.leftStickButtonOnce() || gg2.startOnce()) {
//            robot.outtake.launcher.toggleZone();
//        }


        if(gg.bOnce()) {
            if(robot.outtake.outtakeState == Outtake.OuttakeState.IDLE ) {
                robot.outtake.start_feed_precise(OuttakePositions.farLaunchVelocity,OuttakePositions.farLaunchTilt);

            } else if(isReadyingFlywheel) {
                isReadyingFlywheel = false;
                robot.outtake.start_feed_precise(OuttakePositions.farLaunchVelocity, OuttakePositions.farLaunchTilt);
            }
            else{
                robot.outtake.outtakeState = Outtake.OuttakeState.STOP;
            }
        }


        if(gg.startOnce()) {
            if(robot.outtake.turret.turretState == Turret.TurretState.TRACKING) {
                robot.outtake.turret.turretState = Turret.TurretState.FIXED_ANGLE;
            }else {
                robot.outtake.turret.turretState = Turret.TurretState.TRACKING;
            }
        }


    }
}