package org.firstinspires.ftc.teamcode.opmodes.teleops

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.Gamepad.LedEffect
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.teamcode.opmodes.autos.TestVars
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.robot.RumbleStrength
import org.firstinspires.ftc.teamcode.robot.Side
import org.firstinspires.ftc.teamcode.opmodes.teleops.Lift.*


private enum class States {
    OPEN, CLOSE, UP, DOWN, LOCKED, UNLOCKED, LAUNCHED, STAGED
}
private enum class Lift {
    LOWER, READY, CLOSE, GUARANOP
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

        var LiftState = READY


        waitForStart()
        /* END - STARTING ROBOT */

        while (opModeIsActive()){
            telemetry.addData("Loop Time", timer.milliseconds())
            timer.reset()

            /* DRIVER 2 */


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


            /////////////////////////////////////



            when (LiftState) {
                LOWER -> {

                    ROBOT.LIFT.mode = DcMotor.RunMode.RUN_USING_ENCODER
                    ROBOT.LG.position = TestVars.LGClose
                    ROBOT.RG.position = TestVars.RGClose
                    ROBOT.WRIST.position = TestVars.WristLevelPos
                    WristState = States.DOWN
                    ROBOT.LIFT.targetPosition = 0
                    ROBOT.LIFT.mode = DcMotor.RunMode.RUN_TO_POSITION
                    ROBOT.LIFT.velocity = 1200.0

                    if (ROBOT.LIFT.currentPosition <= 100) {
                        if (ROBOT.LIFT.currentPosition < 0) {
                            ROBOT.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                        }
                        ROBOT.LG.position = TestVars.LGOpen
                        ROBOT.RG.position = TestVars.RGOpen
                        RGSTATE = States.OPEN
                        LGSTATE = States.OPEN
                        LiftState = READY
                        ROBOT.LIFT.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                    }
                }
                CLOSE -> {
                    RGSTATE = States.CLOSE
                    LGSTATE = States.CLOSE
                    ROBOT.LG.position = TestVars.LGClose
                    ROBOT.RG.position = TestVars.RGClose
                    LiftState = READY
                }
                GUARANOP -> {
                    RGSTATE = States.OPEN
                    LGSTATE = States.OPEN
                    ROBOT.LG.position = TestVars.LGOpen
                    ROBOT.RG.position = TestVars.RGOpen
                    LiftState = READY
                }
                READY -> {
//                    if (gamepad2.dpad_down && !d2Clone.dpad_down){
//                        if(gamepad2.dpad_down) {
//                            LiftState = LOWER
//                        }
//                    }

                    if (gamepad2.dpad_up && !d2Clone.dpad_up){
                        if(gamepad2.dpad_up) {
                            LiftState = CLOSE
                        }
                    }

                    ROBOT.LIFT.power = -gamepad2.left_stick_y.toDouble()

                    if (gamepad2.dpad_down && !d2Clone.dpad_down) {
                        LiftState = GUARANOP
                    }

                    /* LEFT GRIPPER TOGGLE */
                    if (gamepad2.left_bumper && !d2Clone.left_bumper) {
                        if (gamepad2.left_bumper) {
                            if (LGSTATE == States.OPEN) {
                                ROBOT.LG.position = TestVars.LGClose
                                LGSTATE = States.CLOSE
                            }
                            else {
                                ROBOT.LG.position = TestVars.LGOpen
                                LGSTATE = States.OPEN
                            }
                        }
                    }
                    /* END LEFT GRIPPER TOGGLE */

                    /* RIGHT GRIPPER TOGGLE */
                    if (gamepad2.right_bumper && !d2Clone.right_bumper) {
                        if (gamepad2.right_bumper) {
                            if (RGSTATE == States.OPEN) {
                                ROBOT.RG.position = TestVars.RGClose
                                RGSTATE = States.CLOSE
                            }
                            else {
                                ROBOT.RG.position = TestVars.RGOpen
                                RGSTATE = States.OPEN
                            }
                        }
                    }
                    /* END RIGHT GRIPPER TOGGLE */

                    if (gamepad2.triangle && !d2Clone.triangle) {
                        if (gamepad2.triangle) {
                            if (WristState == States.UP) {
                                ROBOT.WRIST.position = TestVars.WristLevelPos
                                WristState = States.DOWN
                            }
                            else if (WristState == States.DOWN) {
                                ROBOT.WRIST.position = TestVars.WristTop
                                WristState = States.UP
                            }
                        }
                    }
                }
                else -> {
                    LiftState = READY
                }
            }

            /////////////////////////////////////


            /* TOGGLE WRIST POSITION */

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
            /* END ELEVATOR CONTROLS */

            /* DRIVETRAIN SPEED CONTROL */



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
            /* END - DRIVETRAIN SPEED CONTROL */

            /* ACTION LOOP */

            d1Clone.copy(gamepad1)
            d2Clone.copy(gamepad2)

            ROBOT.gamepadDrive(gamepad1, m)


            telemetry.addData("LIFT STATE", LiftState)
            telemetry.addData("Launcher Safety", Safety)
            telemetry.addData("Wrist State", WristState)
            telemetry.addData("Wrist Count", wristCount)
            telemetry.addData("Left Claw", LGSTATE)
            telemetry.addData("Right Claw", RGSTATE)

            telemetry.update()
            /* END - ACTION LOOP */
        }

        
    }

    fun closeClaws() {
        Robot(hardwareMap).LG.position = TestVars.LGClose
        Robot(hardwareMap).RG.position = TestVars.RGClose
    }

    /* END - FUNCTIONS */
}