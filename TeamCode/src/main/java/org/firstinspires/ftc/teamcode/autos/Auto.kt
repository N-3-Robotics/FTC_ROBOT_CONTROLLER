@file:Suppress("unused")

package org.firstinspires.ftc.teamcode.autos

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.robot.Robot

@Disabled
@Autonomous(name = "Cone Parking", group = "Autonomous")
class Auto: LinearOpMode() {


    override fun runOpMode() {
        val ROBOT = Robot(hardwareMap)
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        

        while(!opModeIsActive() && !isStopRequested) {

        }
    }
}
