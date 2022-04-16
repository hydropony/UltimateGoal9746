package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.PIDcontroller;

@Config
@TeleOp
public class PIDtry extends LinearOpMode {
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static int target = 0;
    FtcDashboard dashboard;
    Robot20 R = new Robot20();

    @Override
    public void runOpMode() throws InterruptedException {
        R.init(this, false);

        PIDcontroller controller = new PIDcontroller(kP, kI, kD);
        controller.target = target;
        controller.isOutputBounded = true;
        controller.maxOutput = 1;

        waitForStart();

        while (!isStopRequested())
        {
            R.shoot_m.setPower(controller.getAction(R.shoot_m.getCurrentPosition()));
        }
        R.wobble_m.setPower(0);
    }
}