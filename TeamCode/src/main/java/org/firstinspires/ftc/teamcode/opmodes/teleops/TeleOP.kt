package org.firstinspires.ftc.teamcode.opmodes.teleops

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.Gamepad.LedEffect
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.teamcode.opmodes.autos.TestVars
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.robot.RumbleStrength
import org.firstinspires.ftc.teamcode.robot.Side


private enum class States {
    OPEN, CLOSE, UP, DOWN, LOCKED, UNLOCKED, LAUNCHED, STAGED
}

@TeleOp(name = "TeleOp")
class TeleOP: LinearOpMode() {
    override fun runOpMode() {


        /* INITIALIZATION */
        val timer = ElapsedTime()


        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val ROBOT = Robot(hardwareMap)

        var d1Clone: Gamepad = Gamepad()
        var d2Clone: Gamepad = Gamepad()

        var wristCount = 0

        d1Clone.copy(gamepad1)
        d2Clone.copy(gamepad2)

        var m = 0.5

        var LGSTATE = States.CLOSE
        var RGSTATE = States.CLOSE
        var WristState = States.DOWN
        var LaunchState = States.STAGED
        var Locker = States.UNLOCKED
        var Safety = true


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
        // Create effects to indicate to drivers if the claws are open
        val openEffectLeft: LedEffect = LedEffect.Builder()
                .addStep(0.0,1.0,0.0,250)
                .addStep(0.0,0.0,1.0,250)
                .build()
        val openEffectRight: LedEffect = LedEffect.Builder()
                .addStep(1.0,0.0,0.0,250)
                .addStep(0.0,1.0,0.0,250)
                .build()
        val openEffectBoth:LedEffect = LedEffect.Builder()
                .addStep(1.0,0.0,0.0,250)
                .addStep(0.0,0.0,1.0,250)
                .build()

        waitForStart()
        /* END - STARTING ROBOT */

        while (opModeIsActive()){
            telemetry.addData("Loop Time", timer.milliseconds())
            timer.reset()

            /* DRIVER 2 */


            /* LG STATE MACHINE */
            if (LGSTATE == States.CLOSE) {
                ROBOT.LG.position = TestVars.LGClose
            }

            else if (LGSTATE == States.OPEN){
                ROBOT.LG.position = TestVars.LGOpen

            }
            /* END LG STATE MACHINE */

            /* RG STATE MACHINE */
            if (RGSTATE == States.CLOSE) {
                ROBOT.RG.position = TestVars.RGClose
            }

            else if (RGSTATE == States.OPEN) {
                ROBOT.RG.position = TestVars.RGOpen
            }
            /* END RG STATE MACHINE */

            /* END GRIPPER INDICATORS */

            /* WRIST STATE MACHINE */
            if (WristState == States.UP) {
                ROBOT.WRIST.position = TestVars.WristTop

//                if (ROBOT.LIFT.currentPosition < TestVars.AUTODOWN) {
//                    WristState = States.DOWN
//                }

            }
            else if (WristState == States.DOWN) {
                ROBOT.WRIST.position = TestVars.WristLevelPos
            }
            /* WRIST STATE MACHINE */

            /* LOCK STATE MACHINE */
            if(Locker == States.LOCKED){
                ROBOT.LOCK.position = TestVars.LOCKLock
            }
            else if (Locker == States.UNLOCKED){
                ROBOT.LOCK.position = TestVars.LOCKUnlock
            }
            /* END LOCK STATE MACHINE */

            /* LAUNCHER STATE MACHINE */
            if(LaunchState == States.STAGED){
                ROBOT.LAUNCHER.position = TestVars.LAUNCHERStaged
            }
            else if (LaunchState == States.LAUNCHED){
                ROBOT.LAUNCHER.position = TestVars.LAUNCHERLaunch
            }
            /* END LAUNCHER STATE MACHINE */

            /* SAFETY STATE MACHINE */
            if (Safety) {
                ROBOT.SAFETY.position = TestVars.SAFETYLocked
            }
            else {
                ROBOT.SAFETY.position = TestVars.SAFETYUnlocked
            }
            /* END SAFETY STATE MACHINE */

            /* TOGGLE WRIST POSITION */
            if (gamepad2.triangle && !d2Clone.triangle) {
                wristCount++
                if (gamepad2.triangle) {
                    if (WristState == States.UP)
                        WristState = States.DOWN
                    else
                        WristState = States.UP
                }
            }
           /* END TOGGLE WRIST POSITION*/

            /* TOGGLE LOCK */
            if (gamepad2.square && !d2Clone.square) {
                if (gamepad2.square) {
                    if (Locker == States.UNLOCKED)
                        Locker = States.LOCKED
                    else
                        Locker = States.UNLOCKED
                }
            }
            /* END TOGGLE LOCK */

            /* LEFT GRIPPER TOGGLE */
            if (gamepad2.left_bumper && !d2Clone.left_bumper) {
                if (gamepad2.left_bumper) {
                   if (LGSTATE == States.OPEN)
                       LGSTATE = States.CLOSE
                    else
                        LGSTATE = States.OPEN
                }
            }

            /* END LEFT GRIPPER TOGGLE */

            /* RIGHT GRIPPER TOGGLE */
            if (gamepad2.right_bumper && !d2Clone.right_bumper) {
                if (gamepad2.right_bumper) {
                    if (RGSTATE == States.OPEN)
                        RGSTATE = States.CLOSE
                    else
                        RGSTATE = States.OPEN
                }
            }
            /* END RIGHT GRIPPER TOGGLE */

            /* Launcher Toggle */
            if(gamepad2.circle && !d2Clone.circle){
                if(gamepad2.circle && !Safety){
                    if(LaunchState == States.STAGED)
                        LaunchState = States.LAUNCHED
                    else
                        LaunchState = States.STAGED
                }
            }
            /* End Launcher Toggle */

            /* Safety Toggle */
            if(gamepad2.dpad_up && !d2Clone.dpad_up){
                if(gamepad2.dpad_up){
                    Safety = Safety != true // Safety = the opposite of what it is.
                    haptic(gamepad2, Side.RIGHT)
                }
            }
            /* End Safety Toggle */

            /* NEW LAUNCHER */
            ROBOT.PL.power = if (gamepad2.touchpad) {0.8} else {0.0}
            /* END NEW LAUNCHER */


            /* ELEVATOR CONTROLS */
            if (gamepad2.left_trigger > 0) {
                ROBOT.ELEVATOR.power = -gamepad2.right_stick_y.toDouble()
            }
            else {
                ROBOT.ELEVATOR.power = 0.0
            }
            ROBOT.LIFT.power = -gamepad2.left_stick_y.toDouble()
            /* END ELEVATOR CONTROLS */

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


            telemetry.addData("Launcher Safety", Safety)
            telemetry.addData("Wrist State", WristState)
            telemetry.addData("Wrist Count", wristCount)

            telemetry.update()
            /* END - ACTION LOOP */
        }

        
    }
    /* FUNCTIONS */
    fun haptic(controller: Gamepad, side: Side) {
        Robot(hardwareMap).rumble(controller, side, RumbleStrength.LOW, 300)
    }
    fun closeClaws() {
        Robot(hardwareMap).LG.position = TestVars.LGClose
        Robot(hardwareMap).RG.position = TestVars.RGClose
    }

    /* END - FUNCTIONS */
}