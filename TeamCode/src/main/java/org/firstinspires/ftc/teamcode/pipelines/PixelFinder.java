package org.firstinspires.ftc.teamcode.pipelines;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class PixelFinder extends OpenCvPipeline {

    private String output = "nothing";

    public PixelFinder() {

    }

    // Mat is the image matrix that should be processed.
    @Override
    public Mat processFrame(Mat input) {
        output = "Sample Pipeline Is Running!";
        return input;
    }

    public String getOutput() {
        return output;
    }
}
