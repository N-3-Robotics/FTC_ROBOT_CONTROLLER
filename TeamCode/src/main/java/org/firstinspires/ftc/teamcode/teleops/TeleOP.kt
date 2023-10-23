package org.firstinspires.ftc.teamcode.teleops

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.Gamepad.LedEffect
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.teamcode.autos.TestVars
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.utilities.*
import org.firstinspires.ftc.teamcode.utilities.QOL.Companion.rED

@TeleOp(name = "TeleOp")
class TeleOP: LinearOpMode() {
    override fun runOpMode() {


        /* INITIALIZATION */
        val timer = ElapsedTime()


        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val ROBOT = Robot(hardwareMap)

        var d1Clone: Gamepad = Gamepad()
        var d2Clone: Gamepad = Gamepad()

        d1Clone.copy(gamepad1)
        d2Clone.copy(gamepad2)

        var m = 0.5

        /* END - INITIALIZATION */


        /* STARTING ROBOT */

        // create an LED effect for player one, where the controller blinks blue once, then stays green
        val p1effect: LedEffect = LedEffect.Builder()
                .addStep(0.0, 0.0, 1.0, 250)
                .addStep(0.0, 0.0, 0.0, 125)
                .addStep(1.0, 1.0, 1.0, Gamepad.LED_DURATION_CONTINUOUS)
                .build()

        // create an LED effect for player two, where the controller blinks red twice, then stays green
        val p2effect: LedEffect = LedEffect.Builder()
                .addStep(1.0, 0.0, 0.0, 250)
                .addStep(0.0, 0.0, 0.0, 125)
                .addStep(1.0, 0.0, 0.0, 250)
                .addStep(0.0, 0.0, 0.0, 125)
                .addStep(1.0, 1.0, 1.0, Gamepad.LED_DURATION_CONTINUOUS)
                .build()
        gamepad1.runLedEffect(p1effect)
        gamepad2.runLedEffect(p2effect)
        ROBOT.rumble(gamepad1, Side.BOTH, RumbleStrength.HIGH, 5000)
        ROBOT.rumble(gamepad2, Side.BOTH, RumbleStrength.HIGH, 5000)

        waitForStart()
        /* END - STARTING ROBOT */

        while (opModeIsActive()){
            telemetry.addData("Loop Time", timer.milliseconds())
            timer.reset()

            /* DRIVER 2 */
            if (gamepad2.triangle && !d2Clone.triangle) {
                if (gamepad2.triangle) {
                    ROBOT.WRIST.position = TestVars.WristLevelPos
                }
            }
            if (gamepad2.cross && !d2Clone.cross){
                if (gamepad2.cross){
                    ROBOT.WRIST.position =
                }
            }

            ROBOT.ELEVATOR.power = -gamepad2.right_stick_y.toDouble()
            ROBOT.LIFT.power = -gamepad2.left_stick_y.toDouble()

            /* DRIVETRAIN SPEED CONTROL */
            when {
                gamepad1.cross -> {
                    m = 0.25
                    haptic(gamepad1, Side.RIGHT)
                }
                gamepad1.circle -> {
                    m = 0.5
                    haptic(gamepad1, Side.RIGHT)
                }
                gamepad1.square -> {
                    m = 0.75
                    haptic(gamepad1, Side.RIGHT)
                }
                gamepad1.triangle -> {
                    m = 1.0
                    haptic(gamepad1, Side.RIGHT)
                }
            }
            /* END - DRIVETRAIN SPEED CONTROL */

            /* ACTION LOOP */

            d1Clone.copy(gamepad1)
            d2Clone.copy(gamepad2)

            ROBOT.gamepadDrive(gamepad1, m)

            telemetry.addData("Elevator Position", ROBOT.ELEVATOR.currentPosition)
            telemetry.addData("Lift Position", ROBOT.LIFT.currentPosition)
            telemetry.addData("Global Heading", ROBOT.IMU.angularOrientation.toAxesOrder(AxesOrder.XYZ).thirdAngle)

            telemetry.update()
            /* END - ACTION LOOP */
        }

        
    }
    /* FUNCTIONS */
    fun haptic(controller: Gamepad, side: Side) {
        Robot(hardwareMap).rumble(controller, side, RumbleStrength.LOW, 300)
    }
    /* END - FUNCTIONS */
}