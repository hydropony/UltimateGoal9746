package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot20;

@Disabled
@TeleOp
public class TELE_X extends LinearOpMode {
    Robot20 R = new Robot20();

    @Override
    public void runOpMode() throws InterruptedException {
        R.init(this, false);
        R.attachGamepads(gamepad1, gamepad2);

        waitForStart();

        R.modeW = Robot20.Wobble_mode.STOP;
        R.mode = Robot20.WB_mode.CONTROL;
        R.statr_ang = 90;
        R.WheelBase.start();
        R.WobbleControl.start();
        R.ShooterPID.start();
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
            } else if (Math.abs(gamepad2.right_stick_y) >= 0.1);
                R.modeW = Robot20.Wobble_mode.CONTROL;
            if (gamepad1.right_stick_y >= 0.9){
                R.wb_rotate = false;
            }
            else if (gamepad1.right_stick_y <= -0.9)
                R.wb_rotate = true;
            if (gamepad1.right_stick_button) {
                R.target_angle = 0;
                R.delay(300);
            }
            if (gamepad1.left_stick_button){
                R.field_rotate = !R.wb_rotate;
                R.delay(300);
            }
            R.module_control();
            telemetry.addData("WB_mode (field rotate)", R.field_rotate);
            telemetry.addData("WB_mode (wb rotate)", R.wb_rotate);
            telemetry.addData("Wobble_mode", R.modeW);
            telemetry.addData("isShooting", R.flag_shoot);
            telemetry.addData("isTarget", R.target_shoot);
            telemetry.update();
        }
        R.WobbleControl.interrupt();
        R.WheelBase.interrupt();
        R.ShooterPID.interrupt();
        //R.velocityCheck.interrupt();
    }
}
