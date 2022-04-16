package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//public class TrajBuilder {
//    public static SampleMecanumDrive drive;
//    public static Pose2d lastPose;
//
//    TrajBuilder(SampleMecanumDrive drive) {
//        this.drive = drive;
//    }
//
//    public static Trajectory blshoot(Pose2d currentPose, double shootangle)
//    {
//        lastPose = new Pose2d(-4, 47, Math.toRadians(shootangle));
//        return drive.trajectoryBuilder(currentPose)
//            .lineToLinearHeading(new Pose2d(-4, 47, Math.toRadians(shootangle)))
//            .build();
//    }
//
//    public static Trajectory blstrafe(Pose2d currentPose, double shootangle)
//    {
//        lastPose = new Pose2d(-4, 47, Math.toRadians(shootangle));
//        return drive.trajectoryBuilder(currentPose)
//                .lineToLinearHeading(new Pose2d(-4, 36, 0))
//                .build();
//    }
//}
