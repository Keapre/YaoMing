package org.firstinspires.ftc.teamcode.blob.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Intake.IntakeTransfer;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Util.Globals.Phase;
import org.firstinspires.ftc.teamcode.Util.Info;
import org.firstinspires.ftc.teamcode.Util.Wrapper.GamePadController;
import org.firstinspires.ftc.teamcode.blob.driveTrain.Blob;


@Config
@Autonomous(name = "Straight back and forth blob")
public class StraightBackAndForthBlob extends LinearOpMode {



    public static double fx = 39.37, fy = 0, fh = 0;
    public static double bx = 0, by = 0, bh = 0;
    public static double rx = 0, ry = -20, rh = Math.toRadians(270);
    public static double transError = 2,velocityError = 4;
    public static boolean useVelocity = false;

    enum STATES{
        FORWARD,
        BACKWARDS,
        RIGHT,
        IDLE
    }
    STATES cs =  STATES.IDLE;

    boolean firstTime;
    Robot robot;

    GamePadController gg;

    @Override
    public void runOpMode() throws InterruptedException {
        gg = new GamePadController(gamepad1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Info.phase = Phase.AUTONOMOUS;
        robot = new Robot(this);
        waitForStart();

        cs = STATES.FORWARD;
        firstTime = true;

        while (opModeIsActive()){
            gg.update();
            robot.update();
            robot.outtake.outtakeState = Outtake.OuttakeState.IDLE;
            if(gg.aOnce()) {
                cs = STATES.FORWARD;
                firstTime = true;
            }else if(gg.bOnce()) {
                cs = STATES.BACKWARDS;
                firstTime = true;
            }
            switch (cs){


                case FORWARD:
                    if(firstTime) {
                        robot.blob.setTargetPosition(fx, fy, Math.toRadians(fh));
                        firstTime = false;
                    }
                    else if((robot.blob.inPosition() && !useVelocity) || (robot.blob.inPositionVelocity(velocityError,transError) && useVelocity)) {
                        cs = STATES.IDLE;
                        firstTime = true;
                    }
                    break;

                case BACKWARDS:
                    if(firstTime) {
                        robot.blob.setTargetPosition(bx, by, Math.toRadians(bh));
                        firstTime = false;
                    }
                    else if((robot.blob.inPosition() && !useVelocity) || (robot.blob.inPositionVelocity(velocityError,transError) && useVelocity)) {
                        cs = STATES.IDLE;
                        firstTime = true;
                    }
                    break;

                case IDLE:
                    break;

            }

            telemetry.addData("cs",cs.name());
            telemetry.addData("x", robot.blob.odo.getX());
            telemetry.addData("y", robot.blob.odo.getY());
            telemetry.addData("xPower",robot.blob.xPower);
            telemetry.addData("yPower",robot.blob.yPower);
            telemetry.addData("headingPower",robot.blob.headingPower);
            telemetry.addData("velocity",robot.blob.odo.getSpeedTranslational());
            telemetry.addData("error x",robot.blob.targetX - robot.blob.odo.x);
            telemetry.addData("error y",robot.blob.targetY - robot.blob.odo.y);
            telemetry.addData("error head",robot.blob.error);
            telemetry.addData("heading", robot.blob.odo.getHeading());
            telemetry.addData("target Heading", robot.blob.targetHeading);
            telemetry.addData("rotation (power)", robot.blob.rotation);
            telemetry.addData("error", robot.blob.error);
            telemetry.addData("real Heading", robot.blob.realHeading);
            telemetry.addData("real heading degrees", Math.toDegrees(robot.blob.realHeading));

            telemetry.update();
        }
    }
}