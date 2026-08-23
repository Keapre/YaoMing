package org.firstinspires.ftc.teamcode.OpMode.robotNou;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Util.Wrapper.DigitalWrapper;


@Config
@TeleOp(name = "Distance Sensor test", group = "Pedro Pathing")
public class BeamBrakeTest extends LinearOpMode {
    public static String beamBrakeName = "laserLeft";
    public static String beamBrakeName2 = "laserRight";
    public static String beamBrakeName3 = "laserTransfer";
    DigitalChannel rawBeamBrake;
    DigitalChannel rawBeamBrake2;
    DigitalChannel rawBeamBrake3;

    DigitalWrapper sensor;
    boolean currentValue = false;
    @Override
    public void runOpMode() throws InterruptedException {
        sensor = new DigitalWrapper(hardwareMap, beamBrakeName);
        rawBeamBrake = hardwareMap.get(DigitalChannel.class, beamBrakeName);
        rawBeamBrake.setMode(DigitalChannel.Mode.INPUT);

        rawBeamBrake2 = hardwareMap.get(DigitalChannel.class, beamBrakeName2);
        rawBeamBrake2.setMode(DigitalChannel.Mode.INPUT);

        rawBeamBrake3 = hardwareMap.get(DigitalChannel.class, beamBrakeName3);
        rawBeamBrake3.setMode(DigitalChannel.Mode.INPUT);
        waitForStart();
        while (opModeIsActive()){
            currentValue = sensor.getValue();
            telemetry.addData("Distance Sensor Wrapper state", currentValue);
            telemetry.addData("Distance Sensor Wrapper state raw", sensor.getRaw());

            telemetry.addData("Distance Sensor  raw state", rawBeamBrake.getState());
            telemetry.addData("Distance Sensor raw state raw", rawBeamBrake.getState());

            telemetry.addData("Distance Sensor 2 raw state", rawBeamBrake2.getState());
            telemetry.addData("Distance Sensor 2 raw state raw", rawBeamBrake2.getState());

            telemetry.addData("Distance Sensor 3 raw state", rawBeamBrake3.getState());
            telemetry.addData("Distance Sensor 3 raw state raw", rawBeamBrake3.getState());

            telemetry.update();
        }
    }
}