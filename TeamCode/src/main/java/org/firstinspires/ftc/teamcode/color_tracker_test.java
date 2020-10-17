package org.firstinspires.ftc.teamcode;

import android.location.Location;

import com.disnodeteam.dogecv.math.Circle;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import static android.icu.lang.UCharacter.GraphemeClusterBreak.L;

public class color_tracker_test extends OpenCvPipeline {
    Telemetry telemetry;
            Mat mat = new Mat();
            public enum Location{
                yes,
                no,
            }
            private Location isItFound;
            static final Rect MAIN_ROI = new Rect(
                    new Point(40, 40),
                    new Point(60, 60)
            );
            static double PERCENT_COLOR_THRESHOLD = 0.4;
    public color_tracker_test(Telemetry t) { telemetry = t;}

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(0,0,0);
        Scalar highHSV = new Scalar(255,255,255);
        Core.inRange(mat, lowHSV, highHSV, mat );
        Mat main = mat.submat(MAIN_ROI);

        double mainValue = Core.sumElems(main).val[0] / MAIN_ROI.area() / 255;
        main.release();
        telemetry.addData("mainValue",(int) Core.sumElems(main).val[0]);
        telemetry.addData("mainPercent",Math.round(mainValue * 100)+ "%");

        boolean stoneMain = mainValue > PERCENT_COLOR_THRESHOLD;

        if(stoneMain) {
            isItFound = Location.yes;
            //add telemetry for trackers here
        }
        else{
            isItFound = Location.no;
        };
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);
        Scalar colorNotFound = new Scalar(255, 0, 0);
        Scalar colorFound = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, MAIN_ROI, isItFound == Location.yes? colorFound:colorNotFound);


        return mat;
        }
    public Location getLocation() {
        return isItFound;
    }
}
