package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
//import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot20;
import org.firstinspires.ftc.teamcode.vision.ContourRingPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.ContourRingPipeline;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Config
@Autonomous(group = "b")
public class BlueWallAuto extends LinearOpMode {
    Robot20 R = new Robot20();
    public static double shootangle = 0;
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

        Pose2d startPose = new Pose2d(-60, 48, 0);

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
                Trajectory trajectoryDown = drive.trajectoryBuilder(startPose, 0)
                        .splineToSplineHeading(new Pose2d(-0, 60, 0), 0)
                        .build();

                Trajectory downtoshoot = drive.trajectoryBuilder(trajectoryDown.end())
                        .lineToLinearHeading(new Pose2d(-7, 48, Math.toRadians(shootangle)))
                        .build();

                Trajectory downpark = drive.trajectoryBuilder(downtoshoot.end())
                        .lineToLinearHeading(new Pose2d(13,36, 0))
                        .build();
                drive.followTrajectory(trajectoryDown);
                R.autoHigh();
                drive.followTrajectory(downtoshoot);
                R.delay(500);
                R.shoot();
                R.delay(500);
                R.shoot();
                R.delay(500);
                R.shoot();
                R.delay(500);
                R.shoot();
                R.stopAutoShoot();
                R.delay(15000);
                drive.followTrajectory(downpark);
                Robot20.statr_ang = R.heading();
                break;
            case ONE:
                Trajectory trajectoryMid = drive.trajectoryBuilder(startPose)
                        .splineToSplineHeading(new Pose2d(-24, 52, 0), 0)
                        .splineToSplineHeading(new Pose2d(24, 45, Math.toRadians(-20)), Math.toRadians(-20))
                        .build();

                Trajectory midtoshoot = drive.trajectoryBuilder(trajectoryMid.end())
                        .lineToLinearHeading(new Pose2d(-5 , 48, Math.toRadians(-shootangle)))
                        .build();

                Trajectory midtozah = drive.trajectoryBuilder(midtoshoot.end())
                        .lineToLinearHeading(new Pose2d(-5 , 35, 0))
                        .build();

                Trajectory midtoshoot2 = drive.trajectoryBuilder(midtozah.end())
                        .lineToLinearHeading(new Pose2d(-8 , 48, Math.toRadians(-shootangle)))
                        .build();

                Trajectory midpark = drive.trajectoryBuilder(midtoshoot2.end())
                        .lineToLinearHeading(new Pose2d(12, 58, 0))
                        .build();

                drive.followTrajectory(trajectoryMid);
                R.autoHigh();
                drive.followTrajectory(midtoshoot);
                R.shoot();
                R.delay(100);
                R.shoot();
                R.delay(100);
                R.shoot();
                drive.followTrajectory(midtozah);
                R.encX.setPower(-1);
                R.delay(20);
                back(10, 0.3, drive);
                R.delay(20);
                forward(10, 0.3, drive);
                drive.followTrajectory(midtoshoot2);
                R.encX.setPower(0);
                R.delay(20);
                R.shoot();
                R.delay(20);
                R.stop_shooting();
                drive.followTrajectory(midpark);
                Robot20.statr_ang = R.heading();
                break;
            case FOUR:
                Trajectory trajectoryHigh = drive.trajectoryBuilder(startPose)
                        .splineToSplineHeading(new Pose2d(-24, 52, 0), 0)
                        .splineToSplineHeading(new Pose2d(48, 59, 0), 0)
                        .build();

                Trajectory hightoshoot = drive.trajectoryBuilder(trajectoryHigh.end())
                        .lineToLinearHeading(new Pose2d(-4, 49, 0))
                        .build();

                Trajectory hightozah = drive.trajectoryBuilder(hightoshoot.end())
                        .lineToLinearHeading(new Pose2d(-4 , 36, 0))
                        .build();

                Trajectory hightozah1 = drive.trajectoryBuilder(new Pose2d(-12.5, 35))
                        .lineToLinearHeading(new Pose2d(-4 , 36, 0))
                        .build();

                Trajectory hightozah2 = drive.trajectoryBuilder(new Pose2d(-27, 35))
                        .lineToLinearHeading(new Pose2d(-5 , 36, 0))
                        .build();

                Trajectory highpark = drive.trajectoryBuilder(hightoshoot.end())
                        .lineToLinearHeading(new Pose2d(12, 58, 0))
                        .build();

                ElapsedTime t = new ElapsedTime();
                double t0 = t.milliseconds();
                drive.followTrajectory(trajectoryHigh);
                R.autoHigh();
                drive.followTrajectory(hightoshoot);
                R.shoot();
                R.delay(100);
                R.shoot();
                R.delay(100);
                R.shoot();
                drive.followTrajectory(hightozah);
                //R.encX.setPower(-1);
                R.encX.setPower(-0.95);
                back(7, 0.2, drive);
                forward(0.1, 0.2, drive);
                R.delay(1100);
                back(1.5, 0.2, drive);
                forward(0.1, 0.2, drive);
                R.delay(1100);
                back(2, 0.2, drive);
                forward(0.1, 0.2, drive);
                drive.followTrajectory(hightozah1);
                drive.turn(Math.toRadians(10));
                R.shoot();
                R.delay(100);
                R.shoot();
                R.delay(100);
                R.shoot();
                drive.turn(Math.toRadians(-10));
                if (t.milliseconds() - t0 <= 24000){
                    telemetry.addData("Go to 4th", null);
                    back(22, 0.3, drive);
                    if (t.milliseconds() - t0 <= 26000) {
                        drive.followTrajectory(hightozah2);
                        drive.turn(Math.toRadians(10));
                        R.encX.setPower(0);
                        R.shoot();
                        R.shoot();
                    }
                }
                telemetry.addData("Go to park", null);
                telemetry.update();
                R.stopAutoShoot();
                drive.followTrajectory(highpark);
                Robot20.statr_ang = R.heading();
                break;
        }
        R.ShooterPID.interrupt();
        Robot20.statr_ang = R.heading();
    }

    void back (double x, double p, SampleMecanumDrive drive){
        drive.update();
        x = drive.currentPose.getX() - x;
        while (drive.currentPose.getX() >= x){
            R.set_Power(-p, -p, -p, -p);
            drive.update();
        }
        R.set_Power(0, 0, 0, 0);
        drive.update();
    }
    void forward (double x, double p, SampleMecanumDrive drive){
        drive.update();
        x = drive.currentPose.getX() + x;
        while (drive.currentPose.getX() <= x){
            R.set_Power(p, p, p, p);
            drive.update();
        }
        R.set_Power(0, 0, 0, 0);
        drive.update();
    }
}
