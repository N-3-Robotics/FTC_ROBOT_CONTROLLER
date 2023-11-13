package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.EncoderDrive;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.MotionProfileLocalizerLineDrive;

@Autonomous
public class FarBlue extends LinearOpMode {

    RobotEx robot = RobotEx.getInstance();

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, telemetry);

        /* while (!isStarted()) {
            telemetry.addData("ROTATION: ", sleeveDetection.getParkingPosition());
            telemetry.update();
        } */

        // scan sleeve
        double LGClose = 0.0;
        double LGOpen = 0.1;
        double RGClose = 0.1;
        double RGOpen = 0.0;
        double SAFETYLocked = 0.4;
        double SAFETYUnlocked = 0.0;
        Servo LG = hardwareMap.get(Servo.class, "LG");
        Servo RG = hardwareMap.get(Servo.class, "RG");
        Servo SAFETY = hardwareMap.get(Servo.class, "SAFETY");


        while(!isStarted()) {
            LG.setPosition(LGClose);
            RG.setPosition(RGClose);
            SAFETY.setPosition(SAFETYLocked);
            //telemetry.addData("ROTATION:", sleeveDetection.getParkingPosition());
            telemetry.addData("LG Position:", LG.getPosition());
            telemetry.addData("RG Position:", RG.getPosition());
        }
        // Initialize the robot
        waitForStart();

        // Notify subsystems before loop

        robot.postInit();

        if (isStopRequested()) return;

        // robot.drivetrain.enableAntiTip();
        EncoderDrive drive = new EncoderDrive(this, telemetry);
        MotionProfileLocalizerLineDrive turn = new MotionProfileLocalizerLineDrive(this, telemetry);


//      ONLY PARKING FROM FAR SIDE
        drive.driveForwardFromInchesBB((48 + ((24 - robot.getLength())/2))*3/4);
        turn.turnToAngle(Math.toRadians(-90));
        drive.driveForwardFromInchesBB(72.0 * 3/4);
        turn.turnToAngle(Math.toRadians(-180));
        drive.driveForwardFromInchesBB(48.0 * 3/4);
        turn.turnToAngle(Math.toRadians(-90));
        drive.driveForwardFromInchesBB(24.0 * 3/4);
        LG.setPosition(LGOpen);
        RG.setPosition(RGOpen);
        sleep(2000);
        drive.driveForwardFromInchesBB(-3.0 * 3/4);
    }
}