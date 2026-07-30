package org.firstinspires.ftc.teamcode.Hardware.Outtake;


import com.acmerobotics.dashboard.config.Config;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware.Module;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.Sensors;
import org.firstinspires.ftc.teamcode.Util.Caching.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.Util.Caching.CachingServo;
import org.firstinspires.ftc.teamcode.Util.Controllers.Team254FlywheelController;
import org.firstinspires.ftc.teamcode.Util.Controllers.velocityController;
import org.firstinspires.ftc.teamcode.Util.Filters.LowPassFilter;
import org.firstinspires.ftc.teamcode.Util.HardwareUtils;
import org.firstinspires.ftc.teamcode.Util.Utils;
import org.firstinspires.ftc.teamcode.Util.Wrapper.InterpLUT;

@Config
public class Launcher implements Module {
    CachingDcMotorEx motor1, motor2;

    Servo tilt;

    public static double offsetPower = 0;
    public static double[] Distances = {1, 50, 55.2, 58, 60, 63, 66, 69, 76.5, 85.5, 90.3, 95, 100, 110, 120, 130, 135, 140, 145, 150, 155, 160, 200};
    // Corresponding Velocity values
    public static double[] velValues = {1320, 1320, 1330, 1360, 1370, 1395, 1400, 1420, 1500, 1550, 1570, 1600, 1700, 1700, 1700, 1850, 2005, 2025, 2045, 2060, 2060, 2060, 2060};
    public static double[] hoodValues = {0.1, 0.1, 0.12, 0.12, 0.13, 0.16, 0.19, 0.20, 0.23, 0.27, 0.27, 0.33, 0.34, 0.34, 0.34, 0.37, 0.39, 0.41, 0.44, 0.45, 0.45, 0.45, 0.45};

//    public static double[] Distances = {1, 50, 58, 66,70.5, 74, 82, 90, 98,106,114,  130,135,140,145,150,155,160,200};
//    // Corresponding Velocity values
//    public static double[] velValues = {1300, 1300, 1310, 1350, 1430, 1450, 1510, 1560, 1700,1750,  1850,1850,1900,1950,2000,2040,2040,2040};
//    public static double[] hoodValues = {0.04, 0.04, 0.12, 0.15,0.19, 0.21, 0.23, 0.25, 0.3, 0.36, 0.37,   0.37, 0.38, 0.41, 0.42, 0.44,0.45,0.45,0.45};
//    //

    velocityController velController;

    InterpLUT velocity = new InterpLUT();
    InterpLUT hood = new InterpLUT();

    public enum LauncherState {
        OFF,
        SPIN_UP,
        SHOOT_STARTED,
        IDLE,
        LAUNCHING,
        GO_TO_VEL_HOOD,
        READY_FLYWHEEL,
        RECYCLE,
        TUNE_PID
    }


    Sensors sensors;
    public double target = 0;
    public boolean auto_aim = true;
    public double currentVel = 0;
    public double recycleVelocity = 500;
    public double recycleTilt = 1;
    public static double coffiecntLowPassFilter = 0.4;
    public LauncherState launcherState = LauncherState.OFF;
    public static boolean rebuildTables = false;

    public static boolean use254 = false;
    Team254FlywheelController velocityController254;
    Robot robot;
    LowPassFilter filter;

    private double motor1mAh = 0;
    private double motor2mAh = 0;
    private final ElapsedTime mAhTimer = new ElapsedTime();

    public Launcher(Robot robot, Sensors sensors) {
        this.robot = robot;
        this.motor1 = new CachingDcMotorEx(robot.hw.get(DcMotorEx.class, "shooter1"), 0.002);
        this.motor2 = new CachingDcMotorEx(robot.hw.get(DcMotorEx.class, "shooter2"), 0.002);
        tilt = robot.hw.get(Servo.class, "tilt");
        HardwareUtils.unlock(motor1);
        HardwareUtils.unlock(motor2);
        addData();
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorEx.Direction.FORWARD);
        velController = new velocityController();
        velocityController254 = new Team254FlywheelController();

        filter = new LowPassFilter(coffiecntLowPassFilter);
        this.sensors = sensors;
        velocityController.reset();
        velocityController254.reset();
        //mAhTimer.reset();
    }


    public void goToSpecificValues(double vel, double hood) {
        target = vel;
        target_tilt = hood;
        launcherState = LauncherState.GO_TO_VEL_HOOD;
        updateOffssetHood = false;
    }

    public void goToSpecificValues(Pose pose) {
        target = velocity.get(sensors.getDistanceFromPose(pose));
        target_tilt = hood.get(sensors.getDistanceFromPose(pose));
        launcherState = LauncherState.GO_TO_VEL_HOOD;
        updateOffssetHood = false;
    }

    public void addData() {
        rebuildLutVelFromDashboard();
        rebuildLutHoodFromDashboard();

    }

    public void rebuildLutVelFromDashboard() {
        velocity = new InterpLUT();
        int len = Math.min(Distances.length, velValues.length);
        for (int i = 0; i < len; i++) {
            velocity.add(Distances[i], velValues[i]);
        }
        velocity.createLUT();
    }

    public void rebuildLutHoodFromDashboard() {
        hood = new InterpLUT();
        int len = Math.min(Distances.length, hoodValues.length);
        for (int i = 0; i < len; i++) {
            hood.add(Distances[i], hoodValues[i]);
        }
        hood.createLUT();
    }

    public static double target_tilt = 0.5;
    public static double power;

    public static double hood_offset_close = -0.035;
    public static double hood_offset_far = -0.05;
    public static double offsetTicks = 0;

    public double getHoodPosition() {
        return target_tilt;
    }

    public double getPower() {
        return power;
    }


    public double getTunePidTarget() {
        return tunePidTarget;
    }

    public static boolean useAdaptiveVel = true;
    public static double idleVelocityClose = 0.64;
    public static double idleVelocityFar = 0.75;
    public static double tunePidTarget = 0;
    private double shootingVoltage;

    private double getTargetWithOffset() {
        return target;
    }

    public void setOffsetInch(double inch) {
        distanceOffset += inch;
    }


    public static double maxCloseZone = 1640;
    public static double minCloseZone = 1200;
    public static double accelerationWeight = 1;

    public static double maxFarZone = 2100;
    public static double minFarZone = 1950;
    public static double distanceOffset = 10;
    public static double distanceDefault = 0;


    public static boolean forceDistance = false;
    public static double forcedRobotX = 0; //robot coords nu shooter coords; robot angle nu e fortat
    public static double forcedRobotY = 0;

    public double getOffsetTicks() {
        return offsetTicks;
    }

    public boolean closeMode = true;
    public boolean updateOffssetHood = false;
    double targetDistance;

    public double predictX, predictY;
    public boolean predicting = false;

    @Override
    public void update() {

        //updateEnergyUsage();

        robot.sensors.setFixedTOF(!closeMode);

        if (launcherState != LauncherState.LAUNCHING && launcherState != LauncherState.SHOOT_STARTED) {
            velController.shooting = false;
        } else {
            velController.shooting = true;
        }
        if (rebuildTables) {
            rebuildLutVelFromDashboard();
            rebuildLutHoodFromDashboard();
            rebuildTables = false;
        }

        currentVel = filter.estimate(-robot.blob.returnFrVelocity());

        double shooterX = robot.sensors.getShooterWorldX();
        double shooterY = robot.sensors.getShooterWorldY();
        double[] predP = velController.predictPoint(shooterX, shooterY);
        predictX = predP[0];
        predictY = predP[1];
        if (predP[0] == shooterX && predP[1] == shooterY) {
            // pidPredict off / far side / already inside the zone -> real position.
            predicting = false;
            targetDistance = robot.sensors.getShooterDistanceToBackboard() + distanceOffset + distanceDefault;
        } else {
            predicting = true;
            // Projected onto the close-zone boundary -> pre-spin to the entry distance.
            targetDistance = Math.hypot(robot.sensors.getTargetX() - predP[0],
                    robot.sensors.getTargetY() - predP[1]) + distanceOffset + distanceDefault;
        }

        if (forceDistance) {
            targetDistance = robot.sensors.getShooterDistanceFromRobot(
                    forcedRobotX, forcedRobotY, robot.sensors.getHeading())
                    + distanceOffset + distanceDefault;
        }

        switch (launcherState) {
            case OFF:
                target = 0;
                power = velController.calculate(target, currentVel, sensors.getVoltage());
                motor1.setPower(power);
                motor2.setPower(power);
                break;
            case TUNE_PID:
                target = tunePidTarget;
                power = velController.calculate(tunePidTarget, currentVel, sensors.getVoltage());
                motor1.setPower(power);
                motor2.setPower(power);
                break;
            case IDLE:
                double speed;

                try {
                    if (useAdaptiveVel) {
                        if (closeMode) {
                            target = velocity.get(Utils.minMaxClip(targetDistance, Distances[0], Distances[velValues.length - 1]));
                            target = Utils.minMaxClip(target, minCloseZone, maxCloseZone);
                            target_tilt = hood.get(Utils.minMaxClip(targetDistance, Distances[0], Distances[hoodValues.length - 1]));

                        } else {
                            target = velocity.get(Utils.minMaxClip(targetDistance, Distances[0], Distances[velValues.length - 1]));
                            target = Utils.minMaxClip(target, minFarZone, maxFarZone);
                            target_tilt = hood.get(Utils.minMaxClip(targetDistance, Distances[0], Distances[hoodValues.length - 1]));

                        }
                        if (use254) {
                            power = velocityController254.calculate(getTargetWithOffset(), currentVel, sensors.getVoltage());
                            target_tilt = hood.get(Utils.minMaxClip(targetDistance, Distances[0], Distances[hoodValues.length - 1]));

                        } else {
                            power = velController.calculate(getTargetWithOffset(), currentVel, sensors.getVoltage());
                            target_tilt = hood.get(Utils.minMaxClip(targetDistance, Distances[0], Distances[hoodValues.length - 1]));

                        }
                    } else {
                        if (sensors.isFarZone()) {
                            power = idleVelocityFar;
                        } else {
                            power = idleVelocityClose;
                        }
                    }
                } catch (Exception e) {
                    target = OuttakePositions.defaultVel;
                    target_tilt = 0.3;
                }
                motor1.setPower(power);
                motor2.setPower(power);

                break;
            case SHOOT_STARTED:
                //robot.outtake.turret.backlashYok();
                if (auto_aim) {
                    if (closeMode) {
                        target = velocity.get(Utils.minMaxClip(targetDistance, Distances[0], Distances[velValues.length - 1]));
                        target = Utils.minMaxClip(target, minCloseZone, maxCloseZone);
                    } else {
                        target = velocity.get(Utils.minMaxClip(targetDistance, Distances[0], Distances[velValues.length - 1]));
                        target = Utils.minMaxClip(target, minFarZone, maxFarZone);
                    }
                    target_tilt = hood.get(targetDistance);
                }
                if (use254) {
                    power = velocityController254.calculate(getTargetWithOffset(), currentVel, sensors.getVoltage());
                } else {
                    power = velController.calculate(getTargetWithOffset(), currentVel, sensors.getVoltage());
                }
                snapshotVoltage();
                motor1.setPower(power);
                motor2.setPower(power);
                break;
            case GO_TO_VEL_HOOD:
                if (updateOffssetHood) {
                    target_tilt = hood.get(targetDistance);
                }
                power = velController.calculate(getTargetWithOffset(), currentVel, sensors.getVoltage());
                motor1.setPower(power);
                motor2.setPower(power);
                break;
            case READY_FLYWHEEL:
                try {
                    if (auto_aim) {
                        target = velocity.get(targetDistance);
                        target += offsetPower;
                        target_tilt = hood.get(targetDistance);
                    }
                } catch (Exception e) {
                    robot.op.gamepad1.rumble(250);
                    target = OuttakePositions.defaultVel;
                    target_tilt = 0.5;
                }
                power = velController.calculate(getTargetWithOffset(), currentVel, sensors.getVoltage());
                motor1.setPower(power);
                motor2.setPower(power);
                break;
            case SPIN_UP:
                power = velController.calculate(getTargetWithOffset(), currentVel, sensors.getVoltage());
                motor1.setPower(power);
                motor2.setPower(power);
                if (auto_aim) {
                    try {
                        target_tilt = hood.get(targetDistance);
                    } catch (Exception e) {
                        break;
                    }
                }
                break;
            case LAUNCHING:
                if(auto_aim) {
                    if (robot.sensors.sotm) {
                        try {
                            target = velocity.get(targetDistance);
                            target += offsetPower;
                        } catch (Exception e) {
                            robot.op.gamepad1.rumble(250);
                            robot.outtake.setOuttakeState(Outtake.OuttakeState.IDLE);
                            break;
                        }
                    }
                    target_tilt = hood.get(targetDistance);
                }


                if (use254) {
                    power = velocityController254.calculate(getTargetWithOffset(), currentVel, sensors.getVoltage());
                } else {
                    power = velController.calculate(getTargetWithOffset(), currentVel, shootingVoltage);
                }
                motor1.setPower(power * accelerationWeight);
                motor2.setPower(power * accelerationWeight);
                break;
            case RECYCLE:
                target = recycleVelocity;
                target_tilt = recycleTilt;
                power = velController.calculate(getTargetWithOffset(), currentVel, sensors.getVoltage());
                motor1.setPower(power);
                motor2.setPower(power);
                break;
        }
        tilt.setPosition(target_tilt + pickHoodOffset());
    }

    private void updateEnergyUsage() {
        double dtHours = mAhTimer.seconds() / 3600.0;
        mAhTimer.reset();
        motor1mAh += motor1.getCurrent(CurrentUnit.MILLIAMPS) * dtHours;
        motor2mAh += motor2.getCurrent(CurrentUnit.MILLIAMPS) * dtHours;
    }

    public double getMotor1mAh() {
        return motor1mAh;
    }

    public double getMotor2mAh() {
        return motor2mAh;
    }

    public double getMotor1Amps() {
        return motor1.getCurrent(CurrentUnit.AMPS);
    }

    public double getMotor2Amps() {
        return motor2.getCurrent(CurrentUnit.AMPS);
    }

    public void resetEnergyUsage() {
        motor1mAh = 0;
        motor2mAh = 0;
        mAhTimer.reset();
    }
    public void primeHood() {
        try {
            double dist = robot.sensors.getShooterDistanceToBackboard() + distanceOffset + distanceDefault;
            target_tilt = hood.get(Utils.minMaxClip(dist, Distances[0], Distances[hoodValues.length - 1]));
        } catch (Exception ignored) {
            target_tilt = 0.3;
        }
        tilt.setPosition(target_tilt + pickHoodOffset());
    }

    public void snapshotVoltage() {
        shootingVoltage = sensors.getVoltage();
    }

    public void increaseDecreaseTarget(double delta) {
        target += (delta * 50);
    }

    public void setTarget(double target, double hood_tilt) {
        if (launcherState != LauncherState.TUNE_PID) launcherState = LauncherState.SHOOT_STARTED;
    }

    public void setFlywheel(double target) {
        this.target = target;
        launcherState = LauncherState.SPIN_UP;
    }

    public void toggleZone() {
        closeMode = !closeMode;
    }

    public void setTargetTPS(double targetTPS) {
        this.target = targetTPS;

    }

    public void setTargetHood(double hood_tilt) {
        target_tilt = hood_tilt;
    }

    public void setAuto_aim(boolean state) {
        auto_aim = state;
    }

    public boolean isReady() {
        if (!use254) {
            return Math.abs(currentVel - getTargetWithOffset()) < OuttakePositions.errorVelThreeshold;
        } else {
            return velocityController254.isReady(currentVel, getTargetWithOffset());
        }
    }

    public void autoAimOn(boolean on) {
        auto_aim = on;
    }

    public double getTarget_tilt() {
        return target_tilt;
    }

    public static double SHOOTER_TICKS_PER_REV = 28.0;
    public static double FLYWHEEL_RADIUS_IN = 69.2 / 25.4;
    public static double FLYWHEEL_GEAR_RATIO = 1.0;
    public static double PROJECTILE_TRANSFER_COEFF = 0.7; // tune in dashboard


    public double getProjectileSpeedEstimate() {
        double tps = target;
        if (SHOOTER_TICKS_PER_REV <= 0) return 0.0;
        double motorRevPerSec = tps / SHOOTER_TICKS_PER_REV;
        double flywheelRevPerSec = motorRevPerSec * FLYWHEEL_GEAR_RATIO;
        double rimSpeedInPerSec = 2.0 * Math.PI * FLYWHEEL_RADIUS_IN * flywheelRevPerSec;
        double projSpeed = PROJECTILE_TRANSFER_COEFF * rimSpeedInPerSec;
        if (projSpeed < 0) projSpeed = 0.0;
        return projSpeed;
    }
    public void setUseAdaptiveVel(boolean val){useAdaptiveVel = val;}
    public void setIdleVelocityClose(double val){idleVelocityClose = val;}
    public void setTargetTilt(double tilt){target_tilt = tilt;}
    public double manualOffset = 0;
    public double pickHoodOffset(){
        if(closeMode){
            return hood_offset_close + manualOffset;
        } else {
            return hood_offset_far;
        }
    }
}