package org.firstinspires.ftc.teamcode.OpMode.Tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.blob.localization.GoBildaPinpointDriver;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.IMU;
import com.acmerobotics.dashboard.canvas.Canvas;
import org.firstinspires.ftc.teamcode.Util.Wrapper.TelemetryUtil;

// Import the GoBilda Pinpoint driver
// Make sure you have the GoBildaPinpointDriver.java in your project

@Config
@TeleOp(name = "Localization Test: Pinpoint vs MegaTag2", group = "Testing")
public class Localization extends LinearOpMode {

    // Hardware definition
    private Limelight3A limelight;
    private GoBildaPinpointDriver odo;
    private BNO055IMU imu;




    // Offsets for the Pinpoint Odometry Pods (in MM)
    // TODO: MEASURE THESE ON YOUR ROBOT!
    // X Offset: Forward/Backward distance from center (Positive = Forward)
    // Y Offset: Left/Right distance from center (Positive = Left)
    private static final double ODO_X_OFFSET = -88.5;
    private static final double ODO_Y_OFFSET = 109.35;
    public static double modifier = 0;
    Pose3D botpose;
    Pose3D botPose2;

    @Override
    public void runOpMode() throws InterruptedException {
        TelemetryUtil.setup();

        // ------------------------------------------------------------------
        // 1. Initialize Limelight
        // ------------------------------------------------------------------
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // Switch to pipeline 0 (usually AprilTag)
        limelight.start();

        // ------------------------------------------------------------------
        // 2. Initialize GoBilda Pinpoint
        // ------------------------------------------------------------------
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Set the offsets defined above
        odo.setOffsets(ODO_X_OFFSET, ODO_Y_OFFSET, DistanceUnit.MM);

        // Select encoder resolution
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // Set encoder directions
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        // Reset the Pinpoint to (0,0,0) at start
        odo.resetPosAndIMU();

        // Initialize REV IMU
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(myIMUparameters);
        imu.resetYaw();

        telemetry.addLine("Initialized. Ready to Start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Get odometry position
            odo.update();
            Pose2D odoPosition = odo.getPosition();
            // Get REV IMU yaw
            double imuYaw = odoPosition.getHeading(AngleUnit.DEGREES);

            // Get Limelight MegaTag2 yaw
            limelight.updateRobotOrientation(modifier+imuYaw);
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    botpose = result.getBotpose_MT2();
                    botPose2 = result.getBotpose();
                    telemetry.addData("MegaTag2 Yaw", "%.2f degrees", imuYaw);
                    telemetry.addData("MegaTag2 x M", botpose.getPosition().x);
                    telemetry.addData("MegaTag2 y M", botpose.getPosition().y);
                    telemetry.addData("MegaTag2 x inch", botpose.getPosition().x * 39.37);
                    telemetry.addData("MegaTag2 y inch", botpose.getPosition().y * 39.37);

                    telemetry.addData("MegaTag x M", botPose2.getPosition().x);
                    telemetry.addData("MegaTag y M", botPose2.getPosition().y);
                    telemetry.addData("MegaTag x inch", botPose2.getPosition().x * 39.37);
                    telemetry.addData("MegaTag y inch", botPose2.getPosition().y * 39.37);

//                    try {
//                        Canvas field = TelemetryUtil.getPacket().fieldOverlay();
//                        double mt2_x_in = botpose.getPosition().x * 39.37;
//                        double mt2_y_in = botpose.getPosition().y * 39.37;
//                        double odo_x_in = odoPosition.getX(DistanceUnit.INCH);
//                        double odo_y_in = odoPosition.getY(DistanceUnit.INCH);
//                        double headining = odoPosition.getHeading(AngleUnit.RADIANS);
//                        field.setStroke("#00ff00");
//                        field.strokeCircle(mt2_x_in, mt2_y_in, 7);
//                        TelemetryUtil.packet.put("MT2 (in)", String.format("%.1f, %.1f", mt2_x_in, mt2_y_in));
//                        field.strokeLine(mt2_x_in, mt2_y_in, mt2_x_in + Math.cos(headining), mt2_y_in + Math.sin(headining));
//
//
//                        field.setStroke("#ff0000");
//                        field.strokeCircle(odo_x_in, odo_y_in, 7);
//                        field.strokeLine(odo_x_in, odo_y_in, odo_x_in + Math.cos(headining), odo_y_in + Math.sin(headining));
//                        TelemetryUtil.packet.put("MT2 (in)", String.format("%.1f, %.1f", odo_x_in, odo_y_in));
//
//
//                    } catch (Exception ignored) {
//
//                     }

                }else {
                    botpose = null;
                }
            }else {
                botpose = null;
            }

            // Output data to telemetry
            telemetry.addData("Pinpoint Position", "X: %.2f, Y: %.2f, Heading: %.2f",
                    odoPosition.getX(DistanceUnit.INCH),
                    odoPosition.getY(DistanceUnit.INCH),
                    odoPosition.getHeading(AngleUnit.DEGREES));

            if(botpose!=null) {
                telemetry.addData("Diff x ",(botpose.getPosition().x *39.37) -  odoPosition.getX(DistanceUnit.INCH));
                telemetry.addData("Diff y ",(botpose.getPosition().y *39.37) -  odoPosition.getY(DistanceUnit.INCH));
                telemetry.addData("Diff x megatag1 ",(botPose2.getPosition().x *39.37) -  odoPosition.getX(DistanceUnit.INCH));
                telemetry.addData("Diff y megatag1",(botPose2.getPosition().y *39.37) -  odoPosition.getY(DistanceUnit.INCH));
            }
            telemetry.addData("REV IMU Yaw", "%.2f degrees",imuYaw);
            telemetry.update();
            TelemetryUtil.sendTelemetry();
        }
    }
}