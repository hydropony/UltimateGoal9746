package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot20;
import org.firstinspires.ftc.teamcode.vision.ContourRingPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Config
@Autonomous(group = "r")
public class RedCenterAuto extends LinearOpMode {
    Robot20 R = new Robot20();
    public static double shootangle = -10;
    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution

    public static int HORIZON = 100; // horizon value to tune

    private static final boolean DEBUG = false; // if debug is wanted, change to true

    private static final boolean USING_WEBCAM = true; // change to true if using webcam
    private static final String WEBCAM_NAME = "webcam"; // insert webcam name from configuration if using webcam
    public static double pp = 140.00;

    private ContourRingPipeline pipeline;
    private OpenCvCamera camera;

    @Override
    public void runOpMode() throws InterruptedException {
        R.init(this, true);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, this);

        ContourRingPipeline.Height height;

        Pose2d startPose = new Pose2d(-60, -24, 0);

        int cameraMonitorViewId = this
                .hardwareMap
                .appContext
                .getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );
        if (USING_WEBCAM) {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createWebcam(hardwareMap.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);
        } else {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        }

        camera.setPipeline(pipeline = new ContourRingPipeline(telemetry, DEBUG));

        ContourRingPipeline.CAMERA_WIDTH = CAMERA_WIDTH;

        ContourRingPipeline.HORIZON = HORIZON;

//        ContourRingPipeline.lowerOrange = new Scalar(0.0, pp, 0.0);

        camera.openCameraDeviceAsync(() -> camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT));

        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(camera, 30);
        dashboard.setTelemetryTransmissionInterval(500);

        R.delay(3000);

        while (!isStarted()) {
            height = pipeline.height;
            telemetry.addData("rings", "" + height);
            telemetry.update();
        }

        height = pipeline.height;
        telemetry.addData("rings", "" + height);
        telemetry.update();

        drive.setPoseEstimate(startPose);

        R.ShooterPID.start();

        switch (height) {
            case ZERO:
                Trajectory trajectoryDown = drive.trajectoryBuilder(startPose)
                        .splineToSplineHeading(new Pose2d(-48, -24), 0)
                        .splineToSplineHeading(new Pose2d(0, -48, Math.toRadians(-45)), Math.toRadians(-45))
                        .build();

                Trajectory downtoshoot = drive.trajectoryBuilder(trajectoryDown.end())
                        .lineToLinearHeading(new Pose2d(-1, -22, Math.toRadians(0)))
                        .build();

                Trajectory downpark = drive.trajectoryBuilder(downtoshoot.end())
                        .lineToLinearHeading(new Pose2d(12, -12, 0))
                        .build();

                drive.followTrajectory(trajectoryDown);
                R.autoTarget();
                R.target_vel += 10;
                drive.followTrajectory(downtoshoot);
//                rotateL(11, 0.2);
                drive.turn(Math.toRadians(11));
                R.delay(100);
                R.shoot();
                R.delay(100);
                R.target_vel += 10;
//                rotateL(15, 0.2);
                drive.turn(Math.toRadians(6));
                R.delay(100);
                R.shoot();
                R.delay(100);
                R.target_vel += 10;
//                rotateL(19, 0.2);
                drive.turn(Math.toRadians(5));
                R.shoot();
                R.delay(100);
                R.stop_shooting();
                drive.followTrajectory(downpark);
                break;
            case ONE:
                Trajectory trajectoryMid = drive.trajectoryBuilder(startPose)
                        .splineToSplineHeading(new Pose2d(-24, -23, 0), 0)
                        .splineToSplineHeading(new Pose2d(24, -30, Math.toRadians(-15)), Math.toRadians(-15))
                        .build();

                Trajectory midtoshoot = drive.trajectoryBuilder(trajectoryMid.end())
                        .lineToLinearHeading(new Pose2d(-1, -25, Math.toRadians(0)))
                        .build();

                Trajectory midtozah = drive.trajectoryBuilder(midtoshoot.end())
                        .lineToLinearHeading(new Pose2d(-5 , -36, 0))
                        .build();

                Trajectory midtoshoot2 = drive.trajectoryBuilder(midtozah.end())
                        .lineToLinearHeading(new Pose2d(-5 , -33, Math.toRadians(10)))
                        .build();

                Trajectory midpark = drive.trajectoryBuilder(midtoshoot2.end())
                        .lineToLinearHeading(new Pose2d(12, -12, 0))
                        .build();

                drive.followTrajectory(trajectoryMid);
                R.autoTarget();
                drive.followTrajectory(midtoshoot);
                R.target_vel += 10;
                drive.turn(Math.toRadians(11));
                R.delay(100);
                R.shoot();
                R.delay(100);
                R.target_vel += 10;
                drive.turn(Math.toRadians(6));
                R.delay(100);
                R.shoot();
                R.delay(100);
                R.target_vel += 10;
                drive.turn(Math.toRadians(5));
                R.shoot();
                R.delay(100);
                drive.followTrajectory(midtozah);
                R.encX.setPower(-1);
                R.delay(20);
                back(1500, 0.3);
                R.delay(20);
                R.autoHigh();
                forward(1500, 0.3);
                drive.followTrajectory(midtoshoot2);
                R.encX.setPower(0);
                R.delay(20);
                R.shoot();
                R.delay(20);
                R.stop_shooting();
                drive.followTrajectory(midpark);
                break;
            case FOUR:
                Trajectory trajectoryHigh = drive.trajectoryBuilder(startPose)
                        .splineToSplineHeading(new Pose2d(-24, -23, 0), 0)
                        .splineToSplineHeading(new Pose2d(48, -48, Math.toRadians(-45)), Math.toRadians(-45))
                        .build();

                Trajectory hightoshoot = drive.trajectoryBuilder(trajectoryHigh.end())
                        .lineToLinearHeading(new Pose2d(-1, -25, Math.toRadians(0)))
                        .build();

                Trajectory hightozah = drive.trajectoryBuilder(hightoshoot.end())
                        .lineToLinearHeading(new Pose2d(-5 , -36, 0))
                        .build();

                Trajectory hightoshoot2 = drive.trajectoryBuilder(hightozah.end())
                        .lineToLinearHeading(new Pose2d(-5 , -33, Math.toRadians(10)))
                        .build();

                Trajectory highpark = drive.trajectoryBuilder(hightoshoot2.end())
                        .lineToLinearHeading(new Pose2d(12, -12, 0))
                        .build();

                drive.followTrajectory(trajectoryHigh);
                R.autoTarget();
                drive.followTrajectory(hightoshoot);
                R.target_vel += 10;
                drive.turn(Math.toRadians(11));
                R.delay(100);
                R.shoot();
                R.delay(100);
                R.target_vel += 10;
                drive.turn(Math.toRadians(6));
                R.delay(100);
                R.shoot();
                R.delay(100);
                R.target_vel += 10;
                drive.turn(Math.toRadians(5));
                R.shoot();
                R.delay(100);
                drive.followTrajectory(hightozah);
                R.encX.setPower(-1);
                R.delay(20);
                back(1500, 0.3);
                R.delay(20);
                R.autoHigh();
                forward(1500, 0.3);
                drive.followTrajectory(hightoshoot2);
                R.encX.setPower(0);
                R.delay(20);
                R.shoot();
                R.delay(20);
                R.stop_shooting();
                drive.followTrajectory(highpark);
                break;
        }
        R.ShooterPID.interrupt();
    }

    void back (double x, double p){
        ElapsedTime t = new ElapsedTime();
        double t0 = t.milliseconds();
        while (t.milliseconds() - t0 <= x){
            R.set_Power(-p, -p, -p, -p);
        }
        R.set_Power(0, 0, 0, 0);
    }
    void forward (double x, double p){
        ElapsedTime t = new ElapsedTime();
        double t0 = t.milliseconds();
        while (t.milliseconds() - t0 <= x){
            R.set_Power(p, p, p, p);
        }
        R.set_Power(0, 0, 0, 0);
    }

    void rotateL (double x, double p){
        ElapsedTime t = new ElapsedTime();
        double t0 = t.milliseconds();
        while (R.heading() <= x){
            R.set_Power(p, p, -p, -p);
        }
        R.set_Power(0, 0, 0, 0);
    }

    void rotateR (double x, double p){
        ElapsedTime t = new ElapsedTime();
        double t0 = t.milliseconds();
        while (R.heading() >= x){
            R.set_Power(-p, -p, p, p);
        }
        R.set_Power(0, 0, 0, 0);
    }

    void to_coord(double x, double y, double p, SampleMecanumDrive drive)  {    }
}