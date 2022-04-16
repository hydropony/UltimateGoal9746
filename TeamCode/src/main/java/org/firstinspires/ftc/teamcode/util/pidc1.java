package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class pidc1 {
    private double target;
    private double kP, kI, kD;

    private double ca;
    private double prev_ca;
    private double err;
    private double prev_err;
    private double prev2_err;
    private double dt;
    private double prev_t;
    private double t;
    private double prev_dt;
    private double sum_t;
    private int n;
    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard;

    private boolean debug = false;

    private ElapsedTime T;

    public void setTarget (double target) {
        this.target = target;
    }

    public void startTime () {
        T = new ElapsedTime();
    }

    public void stopTime () {
        T = null;
    }

    public void reset () {
        ca = 0;
        prev_ca = 0;
        err = 0;
        prev_err = 0;
        prev2_err = 0;
        dt = 0;
        prev_t = 0;
        t = 0;
        prev_dt = 0;
        sum_t = 0;
        n = 0;
        stopTime();
    }

    public void stopTimeAndReset () {
        stopTime();
        reset();
    }

    public pidc1 (PIDCoefficients coefs) {
        kP = coefs.p;
        kI = coefs.i;
        kD = coefs.d;
        reset();
    }

    public pidc1 (PIDCoefficients coefs, FtcDashboard dashboard) {
        kP = coefs.p;
        kI = coefs.i;
        kD = coefs.d;
        this.dashboard = dashboard;
        debug = true;
        reset();
    }

    public double getControlActive (double current_val) {
        err = target - current_val;
        t = T.milliseconds();
        dt = t - prev_t;

        ca = prev_ca + kP * (err - prev_err + kI * err * ((sum_t + dt) / (n + 1)) + kD * (((err - prev_err) / dt) - ((prev_err - prev2_err) / prev_dt)));

        if (debug) {
            packet.put("Erorr", err);
            packet.put("ControlActive", ca);
            packet.put("P", err - prev_err);
            packet.put("I", err * ((sum_t + dt) / (n + 1)));
            packet.put("D", ((err - prev_err) / dt) - ((prev_err - prev2_err) / prev_dt));
            dashboard.sendTelemetryPacket(packet);
        }

        prev_ca = ca;
        prev2_err = prev_err;
        prev_err = err;
        prev_dt = dt;
        n += 1;
        sum_t += dt;
        prev_t = t;

        return ca;
    }

    public double getControlActiveFromError (double err) {;
        t = T.milliseconds();
        dt = t - prev_t;

        ca = prev_ca + kP * (err - prev_err + kI * err * ((sum_t + dt) / (n + 1)) + kD * (((err - prev_err) / dt) - ((prev_err - prev2_err) / prev_dt)));

        if (debug) {
            packet.put("Value", target - err);
            packet.put("Erorr", err);
            packet.put("ControlActive", ca);
            packet.put("P", err - prev_err);
            packet.put("I", err * ((sum_t + dt) / (n + 1)));
            packet.put("D", ((err - prev_err) / dt) - ((prev_err - prev2_err) / prev_dt));
            dashboard.sendTelemetryPacket(packet);
        }

        prev_ca = ca;
        prev2_err = prev_err;
        prev_err = err;
        prev_dt = dt;
        n += 1;
        sum_t += dt;
        prev_t = t;

        return ca;
    }
}
