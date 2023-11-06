package org.firstinspires.ftc.teamcode.autos

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.robot.Robot


@Config
object TestVars{

    @JvmField
    var WristLevelPos: Double = 1.0

    @JvmField
    var WristTop: Double = 0.0

    @JvmField
    var LGOpen: Double = 0.0

    @JvmField
    var LGClose: Double = 0.1

    @JvmField
    var RGOpen: Double = 0.1

    @JvmField
    var RGClose: Double = 0.0

    @JvmField
    var AUTODOWN: Int = 999

    @JvmField
    var LOCKLock: Double = 0.0

    @JvmField
    var LOCKUnlock: Double = 1.0

    @JvmField
    var LAUNCHERStaged: Double = 0.3

    @JvmField
    var LAUNCHERLaunch: Double = 1.0

}

@Autonomous(name="Servo Test", group = "Autonomous")
class ServoTest: LinearOpMode() {

    override fun runOpMode() {
        val ROBOT = Robot(hardwareMap)

        waitForStart()

        while (opModeIsActive()) {

            ROBOT.RG.position = TestVars.RGOpen

            ROBOT.LG.position = TestVars.LGOpen

            ROBOT.WRIST.position = TestVars.WristLevelPos

            sleep(500)

        }
    }




}
@Autonomous(name="Servo Tune", group = "Autonomous")
class servoTune: LinearOpMode() {
    override fun runOpMode() {
        val ROBOT = Robot(hardwareMap)

        waitForStart()

        while (opModeIsActive()) {
            ROBOT.LAUNCHER.position = TestVars.LAUNCHERStaged
            ROBOT.LOCK.position = TestVars.LOCKUnlock
            sleep(1000)
            ROBOT.LAUNCHER.position = TestVars.LAUNCHERLaunch
            ROBOT.LOCK.position = TestVars.LOCKLock
        }
    }
}
