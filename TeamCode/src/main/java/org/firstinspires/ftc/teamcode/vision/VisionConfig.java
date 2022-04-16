package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Scalar;

@Config
public class VisionConfig {
    public static int lh = 10;
    public static int ls = 120; //120
    public static int lv = 120;

    public static int hh = 25;
    public static int hs = 255;
    public static int hv = 255;


    public static Scalar lowerOrange = new Scalar(lh, ls, lv);
    public static Scalar upperOrange = new Scalar(hh, hs, hv);
}
