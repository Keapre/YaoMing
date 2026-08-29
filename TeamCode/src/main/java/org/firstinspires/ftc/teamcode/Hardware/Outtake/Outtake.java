package org.firstinspires.ftc.teamcode.Hardware.Outtake;


import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.Hardware.Intake.IntakeTransfer;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.Sensors;

@Config
public class Outtake {
    Robot robot;
    public Launcher launcher;
    ElapsedTime shooterConsistency;
    public Turret turret;
    int cntTransfer = 0;
    public static double transferThreeshold = 1;

    public static boolean updateTurret = true;
    public static boolean updateLauncher = true;
    public boolean shootingWhileMoving = false;

    public enum OuttakeState {
        OFF,
        IDLE,
        SLEEP,
        START_FEEDING_RAPID_FIRE,
        RAPID_FIRE,
        PRECISE_SHOOT_FEEDING,
        PRECISE_SHOOT,
        READY_FLYWHEEL,
        SPECIFIC,
        STOP,
        RECYCLE
    }
    public OuttakeState outtakeState = OuttakeState.IDLE;
    public Outtake(Robot robot, Sensors sensors){
        this.robot = robot;
        turret = new Turret(robot,sensors);
        launcher = new Launcher(robot,sensors);

    }
    public void start_feed_rapid(double target,double hood) {
        launcher.setTarget(target,hood);
        outtakeState = OuttakeState.START_FEEDING_RAPID_FIRE;
        robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.START_TRANSFER);
        cntTransfer = 0;
    }

    public boolean verify_launcher = true;
    public void start_feed_precise(double target,double hood) {
        launcher.setTarget(target,hood);
        outtakeState = OuttakeState.PRECISE_SHOOT_FEEDING;
        cntTransfer = 0;
    }

    public void update() {
        switch (outtakeState) {
            case OFF:
                turret.forceUpdate = false;
                break;
            case IDLE:
                //launcher.launcherState = Launcher.LauncherState.OFF;
                launcher.launcherState = Launcher.LauncherState.IDLE;
                cntTransfer = 0;
                turret.forceUpdate = false;
                break;
            case START_FEEDING_RAPID_FIRE:
                turret.forceUpdate = true;
                if((!verify_launcher || launcher.isReady())  && ((launcher.launcherState == Launcher.LauncherState.SHOOT_STARTED && robot.intakeTransfer.intakeState == IntakeTransfer.IntakeState.INTERMEDIARY_TRANSFER) || launcher.launcherState == Launcher.LauncherState.TUNE_PID)){
                    cntTransfer++;
                }

                if(cntTransfer > transferThreeshold ) {
                    robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.TRANSFER);
                    outtakeState = OuttakeState.RAPID_FIRE;
                    launcher.launcherState = Launcher.LauncherState.LAUNCHING;
                }
                break;
            case RAPID_FIRE:
                launcher.launcherState = Launcher.LauncherState.LAUNCHING;
                break;
            case PRECISE_SHOOT_FEEDING:
                if(launcher.isReady()) {
                    robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.START_TRANSFER);
                    outtakeState = OuttakeState.PRECISE_SHOOT;
                }
                break;
            case PRECISE_SHOOT:
                Log.w("Debug shoot precise","a intrat in case Precise shoot");
                if(!launcher.isReady()) {
                    robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF_OPEN);
                    outtakeState = OuttakeState.PRECISE_SHOOT_FEEDING;
                    Log.w("Debug shoot precise","se intoarce inapoi in feeding");
                }
                break;
            case SPECIFIC:
                launcher.launcherState = Launcher.LauncherState.GO_TO_VEL_HOOD;
                break;
            case READY_FLYWHEEL:
                launcher.launcherState = Launcher.LauncherState.READY_FLYWHEEL;
                break;
            case STOP:
                //launcher.launcherState = Launcher.LauncherState.OFF;
                launcher.launcherState = Launcher.LauncherState.IDLE;
                robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.OFF);
                outtakeState = OuttakeState.IDLE;
                break;
            case RECYCLE:
                launcher.launcherState = Launcher.LauncherState.RECYCLE;
                if (launcher.isReady()) {
                    robot.intakeTransfer.setIntakeState(IntakeTransfer.IntakeState.RECYCLE);
                }
                break;

        }

        if(updateLauncher) launcher.update();
        if(updateTurret) turret.update();
    }

    public void primeAimForAuto() {
        turret.turretState = Turret.TurretState.TRACKING;
        turret.update();
        launcher.primeHood();
    }

    public void specificValues(double vel,double hood) {
        launcher.goToSpecificValues(vel,hood);
    }
    public void specificValues(Pose pose) {
        outtakeState = OuttakeState.SPECIFIC;
        launcher.goToSpecificValues(pose);
    }

    public void flywheelSpin(double target) {
        launcher.setFlywheel(target);
    }
    public boolean isLaunching() {
        return outtakeState != OuttakeState.STOP && outtakeState != OuttakeState.IDLE;
    }

    public void setOuttakeState(OuttakeState outtakeState) {
        this.outtakeState = outtakeState;
    }

    public DcMotorEx getShooterMotor() {
        return launcher.motor1;
    }

    public boolean isShootingWhileMoving() {
        return shootingWhileMoving;
    }

    public void setShootingWhileMoving(boolean enabled) {
        shootingWhileMoving = enabled;
    }
}
