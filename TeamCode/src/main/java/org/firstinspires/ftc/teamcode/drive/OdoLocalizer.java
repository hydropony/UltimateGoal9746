package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction)
 *    |
 *    v
 *    <----( y direction )---->
 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
public class OdoLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 1024;
    public static double WHEEL_RADIUS = 3 / 2.54; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

//    public static double PARALLEL_X = 11 / 2.54; // X is the up and down direction
//    public static double PARALLEL_Y = 0; // Y is the strafe direction
//
    public static double PARALLEL_X = 10.5 / 2.54; // X is the up and down direction 9.8 10.4
    public static double PARALLEL_Y = 0.0 / 2.54; // Y is the strafe direction 1 0

//    public static double PERPENDICULAR_X = 2 / 2.54;
//    public static double PERPENDICULAR_Y = 6 / 2.54;

    public static double PERPENDICULAR_X = 4.8 / 2.54; // 4.2 4.8
    public static double PERPENDICULAR_Y = 5.5 / 2.54; // 6 5.8

    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    private Encoder parallelEncoder, perpendicularEncoder;

    private SampleMecanumDrive drive;

    public OdoLocalizer(HardwareMap hardwareMap, SampleMecanumDrive drive) {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        this.drive = drive;

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "y"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "x"));

        parallelEncoder.setDirection(Encoder.Direction.REVERSE);
        perpendicularEncoder.setDirection(Encoder.Direction.REVERSE);

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return drive.getRawExternalHeading();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCurrentPosition()),
                encoderTicksToInches(perpendicularEncoder.getCurrentPosition())
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getRawVelocity()),
                encoderTicksToInches(perpendicularEncoder.getRawVelocity())
        );
    }

    @NonNull
    @Override
    public Double getHeadingVelocity() {
        return drive.getRawExternalHeading();
    }
}