package org.firstinspires.ftc.teamcode.OpMode.robotNou;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Util.Wrapper.DigitalWrapper;
import org.firstinspires.ftc.teamcode.Util.Wrapper.GamePadController;

import java.util.List;
@Disabled

@Config
@TeleOp(name = "Rapid Intake Test", group = "Pedro Pathing")
public class RapidIntakeTestSuge extends LinearOpMode {

    DcMotorEx leftFront,rightFront,leftBack,rightBack;
    DcMotorEx shooter1,shooter2;
    DcMotorEx intake;
    DcMotorEx trasnfer;
    GamePadController gg;

    public static String nameBeamBrake = "laserTransfer";

    Servo geckoCapac;
    Servo blockerMingi;
    public static double capacPosition = 0.35;

    public static double openBlockerPosition = 0.75;
    public static double closeBlockerPosition = 0.4;

    DigitalWrapper beamBrake3;
    enum IntakeState {
        ON,
        OFF
    }

    enum ServoBlocker {
        ON,
        OFF
    }

    ServoBlocker servoBlocker = ServoBlocker.OFF;
    IntakeState intakeState = IntakeState.OFF;
    public static boolean isLeftFrontReversed = true;
    public static boolean isRightFrontReversed = false;
    public static boolean isLeftBackReversed = true;
    public static boolean isRightBackReversed = false;
    public static boolean isIntakeReversed = false;
    public static boolean isTransferReversed = false;

    public static boolean isShoote1Reversed = false;
    public static boolean isShoote2Reversed = false;
    List<DcMotorEx> motors;
    public static double intakePower = 1;



    @Override
    public void runOpMode() throws InterruptedException {
        gg = new GamePadController(gamepad1);
        waitForStart();

        setUpMotor();
        while (opModeIsActive()){
            gg.update();
            geckoCapac.setPosition(capacPosition);
            if(gg.aOnce()) {
                if(intakeState == IntakeState.OFF) {
                    intakeState = IntakeState.ON;
                } else {

                    intakeState = IntakeState.OFF;
                }
            }
            if(gg.bOnce()) {
                if(servoBlocker == ServoBlocker.OFF) {
                    servoBlocker = ServoBlocker.ON;
                    shooter1.setPower(0);
                    shooter2.setPower(0);
                } else {
                    servoBlocker = ServoBlocker.OFF;
                    shooter1.setPower((isShoote1Reversed ? -1 : 1));
                    shooter2.setPower((isShoote2Reversed ? -1 : 1));
                }

            }
            switch (intakeState){
                case OFF:
                    intake.setPower(0);
                    trasnfer.setPower(0);
                    break;
                case ON:
                    intake.setPower(intakePower * (isIntakeReversed ? -1 : 1));
                    trasnfer.setPower(intakePower * (isTransferReversed ? -1 : 1));

//                    if(!beamBrake3.getValue()) {
//                        trasnfer.setPower(0);
//                    }else {
//                    }
                    break;
            }

            switch (servoBlocker){
                case OFF:
                    blockerMingi.setPosition(closeBlockerPosition);
                    break;
                case ON:
                    blockerMingi.setPosition(openBlockerPosition);
                    break;
            }

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            leftFront.setPower(((y + x + rx) / denominator) * (isLeftFrontReversed ? -1 : 1));
            leftBack.setPower(((y - x + rx) / denominator) * (isLeftBackReversed ? -1 : 1));
            rightFront.setPower(((y - x - rx) / denominator) * (isRightFrontReversed ? -1 : 1));
            rightBack.setPower(((y + x - rx) / denominator) * (isRightBackReversed ? -1 : 1));


            telemetry.addData("intake current",intake.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("transfer current",trasnfer.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("leftBack current",leftBack.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("leftFront current",leftFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("rightFront current",rightFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("rightBack current",rightBack.getCurrent(CurrentUnit.AMPS));

            telemetry.addData("beam brake value",beamBrake3.getValue());
            telemetry.update();
        }
    }

    void setUpMotor(){
        geckoCapac = hardwareMap.get(Servo.class,"geckoCapac");
        geckoCapac.setPosition(capacPosition);

        beamBrake3 = new DigitalWrapper(hardwareMap,nameBeamBrake);
        blockerMingi = hardwareMap.get(Servo.class,"blockerMingi");
        leftFront = hardwareMap.get(DcMotorEx.class,"leftFront");
//        motors.add(leftFront);
        rightFront = hardwareMap.get(DcMotorEx.class,"rightFront");
//        motors.add(rightFront);
        leftBack = hardwareMap.get(DcMotorEx.class,"leftBack");
//        motors.add(leftBack);
        rightBack = hardwareMap.get(DcMotorEx.class,"rightBack");
//        motors.add(rightBack);


        intake = hardwareMap.get(DcMotorEx.class,"intake");
//        motors.add(intake);
        trasnfer = hardwareMap.get(DcMotorEx.class,"transfer");
//        motors.add(trasnfer);

        shooter1 = hardwareMap.get(DcMotorEx.class,"shooter1");
//        motors.add(shooter1);
        shooter2 = hardwareMap.get(DcMotorEx.class,"shooter2");
//        motors.add(shooter2);

//        for(DcMotorEx motor : motors) {
//            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        }

    }
}