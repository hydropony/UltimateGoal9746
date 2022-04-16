package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import static org.firstinspires.ftc.teamcode.vision.VisionConfig.lowerOrange;
import static org.firstinspires.ftc.teamcode.vision.VisionConfig.upperOrange;

//@Config
public class ContourRingPipeline extends OpenCvPipeline {
    public enum Height {
        ZERO,
        ONE,
        FOUR
    }
    public static Height height;
    private Mat mat;
    private Mat ret;
    Telemetry telemetry;
    boolean debug = false;

    /** width of the camera in use, defaulted to 320 as that is most common in examples **/
    public static int CAMERA_WIDTH = 320;

    /** Horizon value in use, anything above this value (less than the value) since
     * (0, 0) is the top left of the camera frame **/
    public static int HORIZON =  (int)((100.0 / 320.0) * CAMERA_WIDTH);

    /** algorithmically calculated minimum width for width check based on camera width **/
    public static double MIN_WIDTH = (30.0 / 320.0) * CAMERA_WIDTH;

    /** if the calculated aspect ratio is greater then this, height is 4, otherwise its 1 **/
    public static double BOUND_RATIO = 0.7;

    public ContourRingPipeline(Telemetry tele, boolean d) {
        height = Height.ZERO;
        ret = new Mat();
        mat = new Mat();
        telemetry = tele;
        debug = d;
    }

    @Override
    public Mat processFrame(Mat input) {
        ret.release();
        ret = new Mat();

        try {
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

            /**checking if any pixel is within the orange bounds to make a black and white mask**/
            Mat mask = new Mat(mat.rows(), mat.cols(), CvType.CV_8UC1); // variable to store mask in
            Core.inRange(mat, lowerOrange, upperOrange, mask);

            /**applying to input and putting it on ret in black or yellow**/
            Core.bitwise_and(input, input, ret, mask);

            /**applying GaussianBlur to reduce noise when finding contours**/
            Imgproc.GaussianBlur(mask, mask, new Size(5.0, 15.0), 0.00);

            /**finding contours on mask**/
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);

            /**drawing contours to ret in green**/
            Imgproc.drawContours(ret, contours, -1, new Scalar(0, 255.0, 0.0), 3);

            /**finding widths of each contour, comparing, and storing the widest**/
            double  maxWidth = 0;
            Rect maxRect = new Rect();
            Iterator<MatOfPoint> iterator = contours.iterator();
            while (iterator.hasNext()) {
                MatOfPoint c = iterator.next();

                MatOfPoint2f copy = new MatOfPoint2f(c.toArray());
                Rect rect = Imgproc.boundingRect(copy);

                double w = rect.width;
                // checking if the rectangle is below the horizon
                if (w > maxWidth && rect.y + rect.height > HORIZON) {
                    maxWidth = w;
                    maxRect = rect;
                }
                c.release(); // releasing the buffer of the contour, since after use, it is no longer needed
                copy.release(); // releasing the buffer of the copy of the contour, since after use, it is no longer needed
            }

            /**drawing widest bounding rectangle to ret in blue**/
            Imgproc.rectangle(ret, maxRect, new Scalar(0, 0.0, 255.0), 2);

            /** drawing a red line to show the horizon (any above the horizon is not checked to be a ring stack **/
            Imgproc.line(
                    ret,
                    new Point(
                            .0,
                            (double)HORIZON
                    ),
                    new Point(
                            (double)CAMERA_WIDTH,
                            (double)HORIZON
                    ),
                    new Scalar(
                            .0,
                            100.0,
                            100.0)
            );

            if (debug && telemetry != null)
                telemetry.addData("Vision: maxW", maxWidth);

            /** checking if widest width is greater than equal to minimum width
             * using Kotlin if expression (Java ternary) to set height variable
             *
             * height = maxWidth >= MIN_WIDTH ? aspectRatio > BOUND_RATIO ? FOUR : ONE : ZERO
             **/
            if (maxWidth >= MIN_WIDTH) {
                Double aspectRatio = (double)maxRect.height / (double)maxRect.width;

                if (debug)
                    telemetry.addData("Vision: Aspect Ratio", aspectRatio);

                if (aspectRatio > BOUND_RATIO)
                    height = Height.FOUR; // height variable is now FOUR
                else
                    height = Height.ONE; // height variable is now ONE
            }
            else {
                height = Height.ZERO;
            }

            if (debug && telemetry != null)
                telemetry.addData("Vision: Height", height);

            // releasing all mats after use
            mat.release();
            mask.release();
            hierarchy.release();
        }
        catch (Exception e) {
            if (telemetry != null)
                telemetry.addData("[ERROR]", e);
//            e.stackTrace.toList().stream().forEach { x -> telemetry.addLine(x.toString()); }
            e.printStackTrace();
        }
        if (telemetry != null)
            telemetry.update();

        return ret;
    }
}