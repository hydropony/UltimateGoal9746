package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot20;

@Config
@TeleOp
public class TELE_B extends LinearOpMode {
    Robot20 R = new Robot20();

    public static double LEFT_TARGET_ANG = -30;
    public static double MID_TARGET_ANG = -20;
    public static double RIGHT_TARGET_ANG = -10;
    @Override
    public void runOpMode() throws InterruptedException {
        R.init(this, false);
        R.attachGamepads(gamepad1, gamepad2);

        waitForStart();

        R.modeW = Robot20.Wobble_mode.STOP;
        R.mode = Robot20.WB_mode.CONTROL;
        R.WheelBase.start();
        R.WobbleControl.start();
//        R.getShooterVelocity.start();
//        R.velocityCheck.start();
        R.ShooterPID.start();
//        Robot20.statr_ang += 90;

        //R.ve+ cityCheck.start();

        while (!isStopRequested()){
            if (gamepad2.dpad_down){
                R.modeW = Robot20.Wobble_mode.DOWN;
                R.delay(300);
            } else if (gamepad2.dpad_right){
                R.modeW = Robot20.Wobble_mode.MID;
                R.delay(300);
            } else if (gamepad2.dpad_up){
                R.modeW = Robot20.Wobble_mode.UP;
                R.delay(300);
            } else if (gamepad2.dpad_left){
                R.modeW = Robot20.Wobble_mode.STOP;
                R.delay(300);
            } else if (Math.abs(gamepad2.right_stick_y) >= 0.1)
                R.modeW = Robot20.Wobble_mode.CONTROL;
            if (gamepad1.dpad_down){
                R.field_rotate = !R.field_rotate;
                R.delay(300);
            }
            if (gamepad2.x) {
                R.wb_rotate = true;
                R.delay(150);
                R.target_angle = 0;
                R.delay(150);
            }
            if (gamepad2.y) {
                R.wb_rotate = true;
                R.delay(150);
                R.target_angle = 20;
                R.delay(150);
            }
            if (gamepad1.dpad_right) {
                R.wb_rotate = true;
                R.delay(150);
                R.target_angle = (float) RIGHT_TARGET_ANG;
                R.delay(150);
            }
            if (gamepad1.dpad_up) {
                R.wb_rotate = true;
                R.delay(150);
                R.target_angle = (float) MID_TARGET_ANG;
                R.delay(150);
            }
            if (gamepad1.dpad_left) {
                R.wb_rotate = true;
                R.delay(150);
                R.target_angle = (float) LEFT_TARGET_ANG;
                R.delay(150);
            }
            if (gamepad1.right_stick_button) {
                Robot20.statr_ang = -R.heading();
                R.delay(300);
            }
            if (Math.abs(gamepad1.left_stick_x) >= 0.05 || Math.abs(gamepad1.left_stick_y) >= 0.05 || gamepad1.left_bumper || gamepad1.right_bumper ||
                    gamepad1.right_trigger >= 0.05 || gamepad1.left_trigger >= 0.05){
                R.wb_rotate = false;
            }
            R.module_control();
            telemetry.addData("WB_mode (field rotate)", R.field_rotate);
            telemetry.addData("WB_mode (wb rotate)", R.wb_rotate);
            telemetry.addData("Wobble_mode", R.modeW);
            telemetry.addData("isShooting", R.flag_shoot);
            telemetry.addData("isTarget", R.target_shoot);
            telemetry.addData("IMU", R.heading() - Robot20.statr_ang);
            telemetry.update();
        }
        R.WobbleControl.interrupt();
        R.WheelBase.interrupt();
        R.ShooterPID.interrupt();
        Robot20.statr_ang = 0;
//        R.velocityCheck.interrupt();
//        R.getShooterVelocity.interrupt();
        //R.velocityCheck.interrupt();
    }
}
