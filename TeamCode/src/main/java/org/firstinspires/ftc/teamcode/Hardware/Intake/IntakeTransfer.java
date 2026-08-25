package org.firstinspires.ftc.teamcode.Hardware.Intake;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware.Module;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.OuttakePositions;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Turret;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.Sensors;
import org.firstinspires.ftc.teamcode.Util.Caching.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.Util.Caching.CachingServo;
import org.firstinspires.ftc.teamcode.Util.HardwareUtils;
import org.firstinspires.ftc.teamcode.Util.Utils;
import org.firstinspires.ftc.teamcode.Util.Wrapper.BinaryDeque;

@Config
public class IntakeTransfer implements Module {
    public CachingDcMotorEx intake, conveyor;
    private Robot robot;
    CachingServo blocker;

    public static boolean useStall = false;
    public boolean stallTriggeredThisLoop = false;
    public static double INTAKE_NOMINAL_VOLTAGE = 12.0;
    public static double TRANSFER_NOMINAL_VOLTAGE = 12.0;
    public static boolean normalizetransfer = false;
    public static boolean normalizeIntake = false;


    public enum IntakeState {
        OFF,
        INTAKE,
        REVERSE,
        START_TRANSFER,
        ReCycleStart,
        ReCycleMid,
        ReCycleEnd,
        POWER_FOR_TIME,
        OFF_OPEN,

        SLEEP,
        TRANSFER,
        PRE_OFF_OPEN,
        HOLD,
        RECYCLE,
        INTERMEDIARY_TRANSFER;

    }

    public enum BlockerState {
        CLOSE,
        OPEN,
        BLOCKER_ACTUALLY_OPEN
    }



    public enum StallCheck {
        IDLE,
        DETECTED,
        CONFIRMING
    }

    public enum ConveyorState {
        REVERSE_LITTLE,
        OFF,
        ON,
        POWER_FOR_TIME,
        TRANSFER,
        recycle1,
        recycle2,
        recycle3,
        reverseTransfer,
        REVERSE
    }

    public StallCheck stallCheck = StallCheck.IDLE;

    public IntakeState intakeState = IntakeState.OFF;
    public IntakeState lastintakeState = IntakeState.OFF_OPEN;
    public BlockerState blockerState = BlockerState.CLOSE;
    public BlockerState lastblockerState = BlockerState.OPEN;
    public ConveyorState conveyorState = ConveyorState.OFF;
    public ConveyorState lastconveyorState = ConveyorState.ON;
    long startSleep = 0;
    double sleeptime = 0;
    public double intakeSensorCounter = 0;
    IntakeState nextState = IntakeState.OFF;
    public IntakeState previousState = IntakeState.OFF;
    private boolean shooterStartRecycle = false;

    private long blockerOpenTriggeredTime = 0;

    BinaryDeque deq = new BinaryDeque();
    ElapsedTime pre_off_open = null;

    public IntakeTransfer(Robot robot, Sensors sensors) {
        this.robot = robot;
        intake = new CachingDcMotorEx(robot.hw.get(DcMotorEx.class, "intake"), 0.007);
        conveyor = new CachingDcMotorEx(robot.hw.get(DcMotorEx.class, "transfer"), 0.007);

        blocker = new CachingServo(robot.hw.get(Servo.class, "blocker"));
        HardwareUtils.unlock(intake);
        HardwareUtils.unlock(conveyor);
        intake.setCurrentAlert(IntakeConstants.intakeAmpsThreshold, CurrentUnit.AMPS);
        intake.setDirection(IntakeConstants.isIntakeReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        conveyor.setDirection(IntakeConstants.isTransferReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

    }

    private boolean two = false;
    private boolean spinUpRecycleWant = false;
    ElapsedTime recycleStartTimer = new ElapsedTime();
    ElapsedTime recycleMidTimer = new ElapsedTime();
    ElapsedTime recycleEndTimer = new ElapsedTime();
    ElapsedTime intakeSeq = new ElapsedTime();
    ElapsedTime transferReverseTimer = new ElapsedTime();
    public double power_time = 0.5;
    public double time_power = 500;
    public double startStallCheckTime = 0;
    public static double stalCheckDuration = 100;
    public boolean beamChecked = false;

    @Override
    public void update() {

        if (intakeState != IntakeState.INTAKE) beamChecked = false;
        switch (intakeState) {
            case OFF:
                blockerState = BlockerState.CLOSE;
                intake.setPower(0);
                conveyorState = ConveyorState.OFF;
                if (robot.sensors.lightColor == Sensors.LightColor.BLUE) {
                    robot.sensors.setLedColor(Sensors.LightColor.OFF);
                }
                break;
            case PRE_OFF_OPEN:
                intake.setPower(0);
                conveyorState = ConveyorState.REVERSE_LITTLE;
                if (pre_off_open == null) {
                    pre_off_open = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
                }
                if (pre_off_open.milliseconds() > 125) {
                    intakeState = IntakeState.OFF_OPEN;
                    pre_off_open = null;
                }
                break;
            case OFF_OPEN:
                blockerState = BlockerState.BLOCKER_ACTUALLY_OPEN;
                intake.setPower(0);
                conveyorState = ConveyorState.REVERSE_LITTLE;

                break;
            case INTAKE:
                robot.sensors.setLedColor(Sensors.LightColor.RED);
                blockerState = BlockerState.CLOSE;
                intake.setPower(IntakeConstants.intakePowerIntake);
                // Run the transfer until the transfer laser is high (ball staged at the top), then stop it.
                if ((robot.sensors.isLaserTransferBlocked() && robot.sensors.getHowLongTransfer() > IntakeConstants.laserTransferStopDelay) || beamChecked) {
                    conveyorState = ConveyorState.OFF;
                    beamChecked = true;
                } else {
                    conveyorState = ConveyorState.ON;
                }
                if (robot.sensors.areAllLasersBlockedForTime() && beamChecked) {
                    robot.op.gamepad1.rumble(250);
                    robot.sensors.setLedColor(Sensors.LightColor.GREEN);
                    pre_off_open = null;
                    intakeState = IntakeState.OFF;
                }


                break;
            case REVERSE:
                blockerState = BlockerState.CLOSE;
                intake.setPower(-IntakeConstants.reversePower);
                conveyorState = ConveyorState.REVERSE;
                break;
            case START_TRANSFER:
                intake.setPower(0);
                conveyorState = ConveyorState.OFF;
                robot.sensors.setLedColor(Sensors.LightColor.BLUE);
                blockerState = BlockerState.BLOCKER_ACTUALLY_OPEN;
                robot.outtake.launcher.snapshotVoltage();
                sleep(IntakeConstants.openBlockerEarlyDelay, IntakeState.INTERMEDIARY_TRANSFER);


                //Log.w("START TRANSFER","previous state: " + previousState + " intakeState " + intakeState);
                break;
            case ReCycleStart:
                robot.outtake.turret.turretState = Turret.TurretState.FIXED_ANGLE;
                if (recycleStartTimer == null) {
                    recycleStartTimer = new ElapsedTime();
                }
                intake.setPower(IntakeConstants.intakeFirstPhase);
                conveyorState = ConveyorState.recycle1;
                robot.outtake.launcher.autoAimOn(false);
                robot.outtake.launcher.setTargetHood(0.7);
                robot.outtake.setOuttakeState(Outtake.OuttakeState.OFF);
                robot.outtake.flywheelSpin(OuttakePositions.recycleSpeed);
                if (robot.outtake.launcher.isReady() && recycleStartTimer.milliseconds() > IntakeConstants.timerRecycleFirstPhase) {
                    intakeState = IntakeState.ReCycleMid;
                    recycleStartTimer = null;
                }
                break;
            case ReCycleMid:
                if (recycleMidTimer == null) {
                    recycleMidTimer = new ElapsedTime();
                }

                if (recycleMidTimer.milliseconds() > IntakeConstants.timerRecycleOpenBlocker) {
                    blockerState = BlockerState.BLOCKER_ACTUALLY_OPEN;
                    conveyorState = ConveyorState.recycle2;
                    intake.setPower(IntakeConstants.intakeSecondPhase);
                }
                if (recycleMidTimer.milliseconds() > IntakeConstants.timerRecycleOpenBlocker + IntakeConstants.powerArmRecycleUp) {
                }
                if (!two &&
                        recycleMidTimer.milliseconds() > IntakeConstants.timerRecycleOpenBlocker + IntakeConstants.timerRecycleOne) {
                    conveyorState = ConveyorState.OFF;
                    intakeState = IntakeState.ReCycleEnd;
                } else if (two && recycleMidTimer.milliseconds() > IntakeConstants.timerRecycleOpenBlocker + IntakeConstants.timerRecycleTwo) {
                    conveyorState = ConveyorState.OFF;
                    intakeState = IntakeState.ReCycleEnd;
                }
                break;
            case ReCycleEnd:
                if (recycleEndTimer == null) {
                    recycleEndTimer = new ElapsedTime();
                }
                if (shooterStartRecycle) {
                    if (spinUpRecycleWant)
                        robot.outtake.setOuttakeState(Outtake.OuttakeState.READY_FLYWHEEL);
                    else
                        robot.outtake.setOuttakeState(Outtake.OuttakeState.IDLE);
                }
                blockerState = BlockerState.CLOSE;
                intake.setPower(IntakeConstants.intakePhase3);
                conveyorState = ConveyorState.reverseTransfer;
                if (recycleEndTimer.milliseconds() > IntakeConstants.timerIntakeEnd) {
                    intake.setPower(IntakeConstants.intakePhase3);
                    conveyorState = ConveyorState.recycle3;
                }
                if (recycleEndTimer.milliseconds() > IntakeConstants.timerIntakeEnd + IntakeConstants.timerIntakeEnd2) {
                    if (!shooterStartRecycle) {
                        shooterStartRecycle = true;
                        robot.outtake.launcher.autoAimOn(true);
                        robot.outtake.turret.turretState = Turret.TurretState.TRACKING;
                    }
                }
                if (recycleEndTimer.milliseconds() > IntakeConstants.timerIntakeEnd + IntakeConstants.timerIntakeEnd2 + IntakeConstants.doneTransfer) {
                    intakeState = IntakeState.OFF;
                }

                break;
            case POWER_FOR_TIME:
                intake.setPower(power_time);
                conveyorState = ConveyorState.POWER_FOR_TIME;
                sleep(time_power, IntakeState.OFF_OPEN);
                break;
            case TRANSFER:
                // Shooting: funnel staged balls into the flywheel. On entry, reverse the intake at
                // transferReversePower for transferReverseTime ms (settle/unjam), then drive it at
                // transferPowerIntake.
                if (previousState != IntakeState.TRANSFER) {
                    transferReverseTimer.reset();
                }
                if (transferReverseTimer.milliseconds() < IntakeConstants.transferReverseTime) {
                    intake.setPower(IntakeConstants.transferReversePower);
                } else {
                    intake.setPower(IntakeConstants.transferPowerIntake);
                }
                conveyorState = ConveyorState.TRANSFER;
                break;
            case SLEEP:
                //Log.w("Debug shoot precise", " " + (System.currentTimeMillis() - startSleep));
                if (System.currentTimeMillis() - startSleep > sleeptime) {
                    intakeState = nextState;
                }
                break;
            case RECYCLE:
                intake.setPower(IntakeConstants.intakePowerRecycle);
                conveyorState = ConveyorState.ON;
                blockerState = BlockerState.OPEN;
                break;

            case INTERMEDIARY_TRANSFER:
                break;
        }


        switch (blockerState) {
            case OPEN:
                if (blockerOpenTriggeredTime == 0) {
                    blockerOpenTriggeredTime = System.currentTimeMillis();
                }
                if (System.currentTimeMillis() - blockerOpenTriggeredTime >= OuttakePositions.blockerOpenDelayMs) {
                    blockerState = BlockerState.BLOCKER_ACTUALLY_OPEN;
                }
                break;
            case BLOCKER_ACTUALLY_OPEN:
                blockerOpenTriggeredTime = 0;
                blocker.setPosition(IntakeConstants.blockerOpen);
                break;
            case CLOSE:
                blockerOpenTriggeredTime = 0;
                blocker.setPosition(IntakeConstants.blockerClose);
                break;
        }

        lastblockerState = blockerState;
        if (conveyorState != lastconveyorState) {
            switch (conveyorState) {
                case REVERSE_LITTLE:
                    setNormalizedTransferPower(IntakeConstants.ConveyerLittle);
                    break;
                case OFF:
                    setNormalizedTransferPower(0);
                    break;
                case recycle1:
                    setNormalizedTransferPower(IntakeConstants.transferFirstPhase);
                    break;
                case recycle2:
                    setNormalizedTransferPower(IntakeConstants.transferSecondPhase);
                    break;
                case recycle3:
                    setNormalizedTransferPower(IntakeConstants.conveyerPhase3);
                    break;
                case ON:
                    setNormalizedTransferPower(IntakeConstants.onPowerConveyer);
                    break;
                case POWER_FOR_TIME:
                    setNormalizedTransferPower(power_time);
                    break;
                case TRANSFER:
                    if (robot.sensors.isFarZone()) {
                        setNormalizedTransferPower(IntakeConstants.transferPowerIntakeFarZone);
                    } else {
                        setNormalizedTransferPower(IntakeConstants.transferPowerTransfer);
                    }
                    break;
                case reverseTransfer:
                    setNormalizedTransferPower(IntakeConstants.reverseConPhase3);
                    break;
                case REVERSE:
                    setNormalizedTransferPower(-IntakeConstants.reversePower);
                    break;
            }
        }
        lastconveyorState = conveyorState;
        previousState = intakeState;


    }

    public void startRecycle(boolean two) {
        this.two = two;
        recycleStartTimer = null;
        recycleMidTimer = null;
        recycleEndTimer = null;
        intakeState = IntakeState.ReCycleStart;
        spinUpRecycleWant = false;
        shooterStartRecycle = false;
    }

    public void spinUpRecycle() {
        spinUpRecycleWant = true;
    }

    public void setIntakeState(IntakeState intakeState) {
        this.intakeState = intakeState;
    }

    public void setBlockerState(BlockerState blockerState) {
        this.blockerState = blockerState;
    }

    public void increaseIntakeServo(double delta) {

    }

    public void setPowerForTime(double power, double time) {
        this.power_time = power;
        this.time_power = time;
        intakeState = IntakeState.POWER_FOR_TIME;
    }

    public boolean isRecycle() {
        return (intakeState == IntakeState.ReCycleStart || intakeState == IntakeState.ReCycleMid || intakeState == IntakeState.ReCycleEnd);
    }

    private void sleep(double time, IntakeState nextState) {
        startSleep = System.currentTimeMillis();
        intakeState = IntakeState.SLEEP;
        sleeptime = time;
        this.nextState = nextState;
    }

    private void setNormalizedTransferPower(double basePower) {

        if (!normalizetransfer) {
            conveyor.setPower(basePower);
            return;
        }
        double voltage = TRANSFER_NOMINAL_VOLTAGE;
        if (robot != null && robot.sensors != null) {
            voltage = robot.sensors.getVoltage();
        }

        double normalized = basePower * (TRANSFER_NOMINAL_VOLTAGE / voltage);
        if (normalizetransfer) {
            conveyor.setPower(normalized);
        } else {
            conveyor.setPower(basePower);
        }
    }

    private void setNormalizedIntakePower(double basePower) {
        double voltage = INTAKE_NOMINAL_VOLTAGE;
        if (robot != null && robot.sensors != null) {
            voltage = robot.sensors.getVoltage();
        }

        double normalized = basePower * (INTAKE_NOMINAL_VOLTAGE / voltage);
        if (normalizeIntake) {
            intake.setPower(normalized);
        } else {
            intake.setPower(basePower);
        }
    }
}
