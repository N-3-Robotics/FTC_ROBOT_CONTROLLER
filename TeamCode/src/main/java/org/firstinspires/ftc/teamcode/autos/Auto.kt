@file:Suppress("unused")

package org.firstinspires.ftc.teamcode.autos

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.pipelines.AprilTagPipeline
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.highPole
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.midPole
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.slightRaise
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvWebcam


@Autonomous(name = "Cone Parking")
class Auto: LinearOpMode() {


    override fun runOpMode() {
        val ROBOT = Robot(hardwareMap)
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        

        while(!opModeIsActive() && !isStopRequested) {

        }
    }
}
