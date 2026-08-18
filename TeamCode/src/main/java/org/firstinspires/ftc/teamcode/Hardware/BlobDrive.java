package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.blob.Blob;
import com.blob.BlobParams;
import com.blob.geometry.PedroAdapter;
import com.blob.localization.SdkPinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Our {@link Blob}, with the two things the in-project copy had that the library does not.
 *
 * <p><b>Pedro poses.</b> Every auto stores its waypoints as {@code com.pedropathing.geometry.Pose},
 * because the same constants feed Pedro. The library speaks {@code com.blob.geometry.Pose}. Rather
 * than wrapping 140-odd call sites in a converter, the overloads live here and the autos are
 * untouched.
 *
 * <p><b>Flywheel velocity.</b> The shooter encoder is wired into the {@code fl} motor's encoder
 * port, so reading flywheel speed means reading a drive motor. The old Blob exposed that as
 * {@code returnFrVelocity()}. It is robot wiring, not path following, so the library has no business
 * knowing about it and it stays here.
 */
@Config
public class BlobDrive extends Blob {

    /** Flips the sign of the flywheel reading. Which way "positive" is depends on the wiring. */
    public static double shooterSign = -1;

    private final DcMotorEx flEncoder;

    public BlobDrive(HardwareMap hardwareMap, BlobParams params, Blob.State initialState) {
        // SdkPinpointLocalizer, not PinpointLocalizer: it drives the SDK's own Pinpoint driver, so
        // the robot config keeps its stock "goBILDA Pinpoint Odometry Computer" entry. blob's
        // bundled driver would need that entry changed to "blob Pinpoint (goBILDA)" on the hub.
        super(hardwareMap, params, new SdkPinpointLocalizer(hardwareMap, params), initialState);
        // Same device the follower drives. hardwareMap hands back the same instance, so this is a
        // second reference, not a second motor.
        flEncoder = hardwareMap.get(DcMotorEx.class, params.leftFrontName);
    }

    /** Flywheel surface velocity, taken off the encoder plugged into the fl port. */
    public double returnFrVelocity() {
        return flEncoder.getVelocity() * shooterSign;
    }

    public void setTargetPosition(Pose target) {
        setTargetPosition(PedroAdapter.fromPedro(target));
    }

    public void setTargetPosition(Pose target, double headingThresholdPercent) {
        setTargetPosition(PedroAdapter.fromPedro(target), headingThresholdPercent);
    }

    /** Curves through {@code intercept} on the way to {@code target}. */
    public void setTargetPosition(Pose target, Pose intercept) {
        setTargetPosition(PedroAdapter.fromPedro(target), PedroAdapter.fromPedro(intercept));
    }

    public void setTargetPosition(Pose target, Pose intercept, double headingThresholdPercent) {
        setTargetPosition(PedroAdapter.fromPedro(target), PedroAdapter.fromPedro(intercept),
                headingThresholdPercent);
    }

    /** Convenience for the autos, which all seed the pose from a Pedro constant. */
    public void setPose(Pose pose) {
        odo.setPose(PedroAdapter.fromPedro(pose));
    }

    /** One smooth path through every waypoint, from Pedro poses. */
    public void setPath(Pose... waypoints) {
        com.blob.geometry.Pose[] converted = new com.blob.geometry.Pose[waypoints.length];
        for (int i = 0; i < waypoints.length; i++) {
            converted[i] = PedroAdapter.fromPedro(waypoints[i]);
        }
        setPath(converted);
    }
}
