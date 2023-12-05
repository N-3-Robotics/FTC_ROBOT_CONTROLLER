package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pipelines.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.pipelines.BlueCubeFinder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class Camera {
    private OpenCvWebcam webcam;
    private HardwareMap hardwareMap;

    private BlueCubeFinder p1;

    private AprilTagPipeline p2;


    public Camera(HardwareMap hw, Telemetry telemetry) { // hardware map from the base class is a parameter
        p1 = new BlueCubeFinder(telemetry); // initialize your pipeline classes
        p2 = new AprilTagPipeline(0.032, 699.491728169, 699.491728169, 327.417804563, 264.843155529);

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

        webcam.setPipeline(p1); // Setting the intial pipeline

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
        webcam.setPipeline(p2);
    }

    public void setContour(){
        webcam.setPipeline(p1);
    }

    // Get information from pipeline
    public String getPipeline1Output(){
        return p1.getOutput();
    }

//    public String getPipeline2Output(){
//        return p2.;
//    }

    // call stop at the end of the opMode.
    public void stop() {
        webcam.stopStreaming();
    }
}