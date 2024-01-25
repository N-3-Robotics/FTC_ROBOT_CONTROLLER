package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opmodes.autos.Alliance;
import org.firstinspires.ftc.teamcode.pipelines.BluePipeline;
import org.firstinspires.ftc.teamcode.pipelines.RedPipeline;
import org.firstinspires.ftc.teamcode.pipelines.depricated.AprilTagPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class Camera {
    private OpenCvWebcam webcam;
    private HardwareMap hardwareMap;

    private BluePipeline blue;

    private RedPipeline red;

    private AprilTagPipeline aTag;

    private Alliance COLOR;


    public Camera(HardwareMap hw, Telemetry telemetry, Alliance color) { // hardware map from the base class is a parameter
        this.COLOR = color;

        blue = new BluePipeline(telemetry); // initialize your pipeline classes
        red = new RedPipeline(telemetry);
        aTag = new AprilTagPipeline(0.032, 699.491728169, 699.491728169, 327.417804563, 264.843155529);

        this.hardwareMap = hw;    //Configure the Camera in hardwaremap
        int cameraMonitorViewId =
                hardwareMap
                        .appContext
                        .getResources()
                        .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // Get camera from hardware map, replace 'camera' with what is in your controlhub
        webcam =
                OpenCvCameraFactory.getInstance()
                        .createWebcam(hardwareMap.get(WebcamName.class, "webcam 1"), cameraMonitorViewId);


        switch (COLOR) {
            case RED:
                webcam.setPipeline(red);
                break;
            case BLUE:
                webcam.setPipeline(blue);
                break;
        }

        webcam.setMillisecondsPermissionTimeout(2500);

        // Streaming Frames
        webcam.openCameraDeviceAsync(
                new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                    }

                    @Override
                    public void onError(int errorCode) {}
                });
    }

    // Switching Between Pipelines
    public void setAprilTag(){
        webcam.setPipeline(aTag);
    }

    public void setContour(){
        switch (COLOR) {
            case RED:
                webcam.setPipeline(red);
                break;
            case BLUE:
                webcam.setPipeline(blue);
                break;
        }
    }

    // Get information from pipeline
    public String getCubeLocation()
    {
        switch (COLOR) {
            case RED:
                return red.getOutput();
            case BLUE:
                return blue.getOutput();
        }
        return "UNKNOWN";
    }

//    public String getPipeline2Output(){
//        return p2.;
//    }

    // call stop at the end of the opMode.
    public void stop() {
        webcam.stopStreaming();
    }
}