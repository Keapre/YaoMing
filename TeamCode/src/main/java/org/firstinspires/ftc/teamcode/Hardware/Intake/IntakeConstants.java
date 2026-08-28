package org.firstinspires.ftc.teamcode.Hardware.Intake;

import com.acmerobotics.dashboard.config.Config;

@Config
public class IntakeConstants {

    /*-----MOTOR POWERS---*/
    public static double intakePowerIntake = 0.8;
    public static double intakePowerIntakeFarZone = 1;
    public static double transferPowerIntake = 0.8;
    public static double transferPowerIntakeFarZone = 0.8;
    public static double reversePower = 1;
    public static double intakePowerRecycle = 1;

    public static double transferPowerTransfer = 0.85;

    public static double transferReverseTime = 15;   // ms
    public static double transferReversePower = -0.2; // intake power during the reverse window

    public static double intakeFirstPhase = 0;
    public static double intakeSecondPhase = 0.5;
    public static double transferFirstPhase = 0;
    public static double transferSecondPhase = 0.5;
    public static double onPowerConveyer = 1;
    public static double ConveyerLittle = -0.0;
    public static double IntakeLittle = 0.05;

    public static double timerRecycleOne = 230;
    public static double timerRecycleTwo = 475;
    public static double powerArmRecycleUp = 100;
    public static double openBlockerEarlyDelay = 50;



    public static double preciseShotPower = 1;




    public static double intakeAmpsThreshold = 2.7;
    public static boolean isIntakeReversed = true;
    public static boolean isTransferReversed = true;

    /*----- SERVO POSITIONS----*/

    public static double capacBleg = 0.62;
    public static double capacRecycle = 0.44;
    public static double capacClosed = 0.8;
    public static double capacReleaseTransfer = 350;

    public static double blockerOpen = 0.305;
    public static double blockerClose = 0.43;

    public static double powerArmLow = 0.48;
    public static double powerArmRecycle = 0.84;
    public static double powerArmIntake = 0.47;
    public static double powerArmVeryLow = 0.47;

    public static double preciseShotDelay = 400;
    public static double laserTransferStopDelay = 100;
    public static double laserRightStopDelay = 100;
    public static double laserLeftStopDelay = 350;
    public static double timerRecycleFirstPhase = 150;
    public static double timerRecycleOpenBlocker = 200;
    public static double doneTransfer = 150;
    public static double timerIntakeEnd= 250;
    public static double sleepTransfer = 0;
    public static double intakeStartTransfer = 100;
    public static double timerIntakeEnd2= 400;
    public static double intakePhase3 = 1;
    public static double conveyerPhase3 = 1;
    public static double reverseConPhase3 = 0.5;

    /*----- SENSOR CONSTANTS----*/
    public static double intakeSensorThreshold = 3;
}