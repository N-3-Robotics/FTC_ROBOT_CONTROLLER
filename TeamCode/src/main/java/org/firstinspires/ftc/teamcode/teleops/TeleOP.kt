package org.firstinspires.ftc.teamcode.teleops

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.outoftheboxrobotics.photoncore.PhotonCore
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.utilities.*
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.SlidesSpeed
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.highPole
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.lowPole
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.midPole
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.slightRaise
import kotlin.math.abs

@TeleOp(name = "TeleOp")
class TeleOP: LinearOpMode() {
    override fun runOpMode() {

        PhotonCore.enable()

        val timer = ElapsedTime()


        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val ROBOT = Robot(hardwareMap)


        var m = 1.0

        while (!opModeIsActive()){
            ROBOT.rumble(gamepad1, Side.BOTH, RumbleStrength.HIGH)
            ROBOT.rumble(gamepad2, Side.BOTH, RumbleStrength.HIGH)
        }

        waitForStart()

        while (opModeIsActive()){
            telemetry.addData("Loop Time", timer.milliseconds())
            timer.reset()


            // Slides Control



            when {
                gamepad1.cross -> {
                    m = 0.25
                }
                gamepad1.circle -> {
                    m = 0.5
                }
                gamepad1.square -> {
                    m = 0.75
                }
                gamepad1.triangle -> {
                    m = 1.0
                }
            }

            when {
                gamepad1.dpad_left -> {
                    ROBOT.turnLeft(90)}
                gamepad1.dpad_right -> {
                    ROBOT.turnRight(90)}
            }


            // Drivetrain Control
            ROBOT.gamepadDrive(gamepad1, m)
            telemetry.addData("Robot Pose", ROBOT.currentPose.toString())
            telemetry.update()
        }

        
    }

}