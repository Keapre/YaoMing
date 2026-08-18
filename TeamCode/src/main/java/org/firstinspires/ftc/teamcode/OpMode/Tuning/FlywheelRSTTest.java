package org.firstinspires.ftc.teamcode.OpMode.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Hardware.Intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.Util.Controllers.RSTFlywheelController;
import org.firstinspires.ftc.teamcode.Util.Filters.LowPassFilter;
import org.firstinspires.ftc.teamcode.blob.constants.BlobConstants;

import java.util.List;

/**
 * Standalone tuner/validator for {@link RSTFlywheelController}, now with ball intake + feed so you
 * can test real firing cycles (the true disturbance test). Launcher/Outtake are NOT used; the intake,
 * transfer, and blocker actuators are driven directly with the robot's own {@link IntakeConstants}
 * values so powers/positions match the real robot. RST (not Launcher) holds the flywheel.
 *
 * <p>Hardware matches {@code FlywheelSysId} / {@code Launcher} for the flywheel: {@code shooter1}/
 * {@code shooter2}, velocity from the "fl" port encoder, low-pass filtered like Launcher.
 *
 * <h3>Controls (gamepad1)</h3>
 * <ul>
 *   <li><b>A</b> -&gt; flywheel target {@link #TARGET_A}, <b>B</b> -&gt; {@link #TARGET_B}, <b>X</b> -&gt; stop wheel.</li>
 *   <li><b>Right trigger</b> (hold) -&gt; INTAKE balls into staging.</li>
 *   <li><b>Right bumper</b> (hold) -&gt; FEED: open blocker + push balls into the spinning wheel (fire).</li>
 *   <li><b>Left bumper</b> (hold) -&gt; REVERSE intake/transfer (unjam).</li>
 * </ul>
 *
 * <h3>Cycle test</h3>
 * Press A or B to spin up, wait until {@code actual} locks on {@code target}, then hold Right bumper
 * to fire. Graph {@code target} vs {@code actual} in the dashboard and watch the dip + recovery on
 * every ball. Tune {@code TAU_CL}/{@code TAU_I} (under {@code RSTFlywheelController}) live.
 */
@Config
@TeleOp(name = "RST TUNING", group = "tuning2")
public class FlywheelRSTTest extends LinearOpMode {

    public static double TARGET_A = 1320;
    public static double TARGET_B = 2060;
    /** Match Launcher's velocity smoothing so tuning transfers 1:1. */
    public static double FILTER_COEF = 0.4;

    public static boolean SHOOTER1_REVERSED = true;
    public static boolean SHOOTER2_REVERSED = false;
    public static boolean FL_ENCODER_REVERSED = true;

    public static double TRIGGER_THRESHOLD = 0.3;

    private enum FeedMode { IDLE, INTAKE, FEED, REVERSE }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule h : hubs) h.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        // ---- Flywheel ----
        DcMotorEx shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        DcMotorEx shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        DcMotorEx flEncoder = hardwareMap.get(DcMotorEx.class, BlobConstants.leftFrontName);
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();

        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter1.setDirection(SHOOTER1_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(SHOOTER2_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        flEncoder.setDirection(FL_ENCODER_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        flEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flEncoder.setPower(0.0);

        // ---- Feeder (driven directly; same names/constants as IntakeTransfer) ----
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        Servo blocker = hardwareMap.get(Servo.class, "blocker");
        Servo powerArm = hardwareMap.get(Servo.class, "powerArm");
        Servo capac = hardwareMap.get(Servo.class, "capac");

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(IntakeConstants.isIntakeReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        transfer.setDirection(IntakeConstants.isTransferReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        RSTFlywheelController rst = new RSTFlywheelController();
        LowPassFilter filter = new LowPassFilter(FILTER_COEF);

        double target = 0.0;

        telemetry.addLine("Flywheel RST Test ready.");
        telemetry.addLine("A/B = target, X = stop wheel.");
        telemetry.addLine("RT = intake, RB = FEED/fire, LB = reverse.");
        telemetry.update();
        waitForStart();
        rst.reset();

        boolean prevA = false, prevB = false, prevX = false;

        while (opModeIsActive()) {
            for (LynxModule h : hubs) h.clearBulkCache();

            // ---- flywheel target ----
            boolean a = gamepad1.a, b = gamepad1.b, x = gamepad1.x;
            if (a && !prevA) { target = TARGET_A; rst.reset(); }
            if (b && !prevB) { target = TARGET_B; rst.reset(); }
            if (x && !prevX) { target = 0.0; }
            prevA = a; prevB = b; prevX = x;

            double vel = filter.estimate(flEncoder.getVelocity());
            double volts = battery.getVoltage();

            double power = (target <= 1.0) ? 0.0 : rst.calculate(target, vel, volts);
            shooter1.setPower(power);
            shooter2.setPower(power);

            // ---- feeder ----
            FeedMode mode;
            if (gamepad1.right_bumper)                         mode = FeedMode.FEED;
            else if (gamepad1.left_bumper)                     mode = FeedMode.REVERSE;
            else if (gamepad1.right_trigger > TRIGGER_THRESHOLD) mode = FeedMode.INTAKE;
            else                                               mode = FeedMode.IDLE;
            applyFeeder(mode, intake, transfer, blocker, powerArm, capac);

            telemetry.addData("target", target);
            telemetry.addData("actual", vel);
            telemetry.addData("error", target - vel);
            telemetry.addData("power", power);
            telemetry.addData("feed", mode);
            telemetry.addData("voltage", volts);
            telemetry.addData("dt_ms", rst.lastDt * 1000.0);
            telemetry.addData("poles", String.format("%.3f / %.3f", rst.lastPole1, rst.lastPole2));
            telemetry.addData("r0/r1/t0", String.format("%.5f / %.5f / %.6f", rst.lastR0, rst.lastR1, rst.lastT0));
            telemetry.update();
        }

        shooter1.setPower(0);
        shooter2.setPower(0);
        intake.setPower(0);
        transfer.setPower(0);
    }

    /** Drive the feeder actuators to match the real IntakeTransfer states, minus the beam sequencing. */
    private void applyFeeder(FeedMode mode, DcMotorEx intake, DcMotorEx transfer,
                             Servo blocker, Servo powerArm, Servo capac) {
        capac.setPosition(IntakeConstants.capacBleg);
        switch (mode) {
            case INTAKE:
                // suck balls into staging: blocker closed so they hold
                blocker.setPosition(IntakeConstants.blockerClose);
                powerArm.setPosition(IntakeConstants.powerArmIntake);
                intake.setPower(IntakeConstants.intakePowerIntake);
                transfer.setPower(IntakeConstants.onPowerConveyer);
                break;
            case FEED:
                // open the gate and push staged balls up into the spinning flywheel (fire)
                blocker.setPosition(IntakeConstants.blockerOpen);
                powerArm.setPosition(IntakeConstants.powerArmVeryLow);
                intake.setPower(IntakeConstants.transferPowerIntake);
                transfer.setPower(IntakeConstants.transferPowerTransfer);
                break;
            case REVERSE:
                blocker.setPosition(IntakeConstants.blockerClose);
                powerArm.setPosition(IntakeConstants.powerArmIntake);
                intake.setPower(-IntakeConstants.reversePower);
                transfer.setPower(-IntakeConstants.reversePower);
                break;
            case IDLE:
            default:
                blocker.setPosition(IntakeConstants.blockerClose);
                powerArm.setPosition(IntakeConstants.powerArmIntake);
                intake.setPower(0);
                transfer.setPower(0);
                break;
        }
    }
}
