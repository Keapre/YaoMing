package org.firstinspires.ftc.teamcode.Hardware.Outtake;


import com.acmerobotics.dashboard.config.Config;

@Config
public class OuttakePositions {
    /*----Launcher ----*/
    public static double FAR_ZONE_X_THRESHOLD = 17;

    /*----Velocity ----*/
    public static double errorVelThreeshold = 35;

    public static double farLaunchVelocity = 2100;

    public static double defaultVel = 1550;


    /*----Tilt ----*/
    public static double farLaunchTilt = 0.40;

    public static double recycleSpeed = 300;

    /*--PIDF VELOCITY--*/
    public static double kP = 0.00768;
    public static double kI = 0;
    public static double kD = 0;
    public static double kV = 0.00038;
    public static double kS = 0.06;


    public static double bbPower = 1.0;
    public static double bbPowerMin = 0;
    public static double bbkV = 0.00035;
    public static double bbKP = 0.006;
    public static double bbThreeshold = 200;

    /*--TURRET--*/
    public static double MIN_TURRET_ANGLE = Math.toRadians(90);
    public static double MAX_TURRET_ANGLE = -Math.toRadians(90);
    public static double MIN_TURRET_POSITION = 0.235;
    public static double MAX_TURRET_POSITION = 0.75;
    public static double MIN_TURRET_RANGE = 0.06;
    public static double MAX_TURRET_RANGE = 0.9;


    /*--BLOCKER--*/
    public static long blockerOpenDelayMs = 0;

}