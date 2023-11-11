package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.EncoderDrive;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.MotionProfileLocalizerLineDrive;
import org.firstinspires.ftc.teamcode.vision.simulatortests.ApriltagDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class FarRedWrong extends LinearOpMode {

    RobotEx robot = RobotEx.getInstance();

    ApriltagDetectionPipeline sleeveDetection;
    OpenCvCamera camera;
    String webcamName = "Webcam 1";

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, telemetry);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new ApriltagDetectionPipeline();
        camera.setPipeline(sleeveDetection);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        /* while (!isStarted()) {
            telemetry.addData("ROTATION: ", sleeveDetection.getParkingPosition());
            telemetry.update();
        } */

        // scan sleeve

        Servo LG = hardwareMap.get(Servo.class, "LG");
        Servo RG = hardwareMap.get(Servo.class, "RG");

        double LGClose = 0.0;
        double LGOpen = 0.2;
        double RGClose = 0.1;
        double RGOpen = -0.1




                ;

        while (!isStarted()) {
            LG.setPosition(LGClose);
            RG.setPosition(RGClose);
            telemetry.addData("ROTATION:", sleeveDetection.getParkingPosition());
            telemetry.addData("RG Position:", RG.getPosition());
            telemetry.addData("LG Position:", LG.getPosition());
            telemetry.update();
        }
        // Initialize the robot
        waitForStart();

        // Notify subsystems before loop

        robot.postInit();

        if (isStopRequested()) return;

        // robot.drivetrain.enableAntiTip();
        EncoderDrive drive = new EncoderDrive(this, telemetry);
        MotionProfileLocalizerLineDrive turn = new MotionProfileLocalizerLineDrive(this, telemetry);


        camera.stopStreaming();


//      ONLY PARKING FROM FAR SIDE
        drive.driveForwardFromInchesBB((48 + ((24 - robot.getLength())/2))*3/4);
        turn.turnToAngle(Math.toRadians(90));
        drive.driveForwardFromInchesBB(72.0*3/4);
        turn.turnToAngle(Math.toRadians(180));
        drive.driveForwardFromInchesBB(20*3/4);
        turn.turnToAngle(Math.toRadians(90));
        drive.driveForwardFromInchesBB(24.0*3/4);
        LG.setPosition(LGOpen);
        RG.setPosition(RGOpen);
        sleep(2000);
        drive.driveForwardFromInchesBB(-3*3/4);
    }
}