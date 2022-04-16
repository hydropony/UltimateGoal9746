package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot20;

@Config
@Autonomous(group = "tuner")
public class CameraTuner extends LinearOpMode {
    Robot20 R = new Robot20();
    @Override
    public void runOpMode() {
        R.init(this, false);
        R.initOCV();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(R.webcam, 30);
        dashboard.setTelemetryTransmissionInterval(500);

        waitForStart();

        boolean red = false;

        while (!isStopRequested()){

            if (gamepad1.x)
                R.vision.set_zone_left(red);
            else if (gamepad1.b)
                R.vision.set_zone_right(red);
            else if (gamepad1.a){
                red = !red;
                R.delay(300);
            }

            telemetry.addData("orange_pix", R.vision.orange_pix);
            telemetry.addData("num_of_rings", R.vision.getNumOfRings());
            telemetry.addData("color is red", red);
            telemetry.update();
        }
    }
}
