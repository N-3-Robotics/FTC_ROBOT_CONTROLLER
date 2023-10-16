package org.firstinspires.ftc.teamcode.autos

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.robot.Robot


@Config
object TestVars{

    @JvmField
    var WristLevelPos: Double = 0.58

    @JvmField
    var LGOpen: Double = 0.35
    @JvmField
    var LGClose: Double = 0.0

    @JvmField
    var RGOpen: Double = 0.0

}

@Autonomous(name="Servo Test")
class ServoTest: LinearOpMode() {

    override fun runOpMode() {
        val ROBOT = Robot(hardwareMap)

        waitForStart()

        while (opModeIsActive()) {

            ROBOT.RG.position = TestVars.RGOpen

            sleep(500)

        }
    }




}
