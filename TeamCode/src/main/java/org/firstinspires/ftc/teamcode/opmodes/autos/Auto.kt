package org.firstinspires.ftc.teamcode.opmodes.autos

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.opmodes.autos.Alliance.*
import org.firstinspires.ftc.teamcode.opmodes.autos.Position.*
import org.firstinspires.ftc.teamcode.robot.Camera
//import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import java.util.LinkedList
import java.util.Queue

enum class Alliance {
    RED, BLUE
}
enum class Position {
    CLOSE, FAR
}

@Autonomous(name = "AUTONOMOUS")
class Auto: LinearOpMode() {

    override fun runOpMode() {
        val ALLIANCE = RED
        val POSITION = FAR

        val CAMERA = Camera(hardwareMap, telemetry, ALLIANCE);

        val DRIVE = SampleMecanumDrive(hardwareMap);
        var StartPose: Pose2d
        var pathToFollow: Queue<TrajectorySequence> = LinkedList()

        var LIFT: DcMotorEx
        var WRIST: Servo
        var LG: Servo
        var RG: Servo

        var Mult: Int = 2

        LIFT = hardwareMap!!.get(DcMotorEx::class.java, "LIFT")
        LIFT.direction = DcMotorSimple.Direction.REVERSE

        WRIST = hardwareMap!!.get(Servo::class.java, "WRIST")
        LG = hardwareMap!!.get(Servo::class.java, "LG")
        RG = hardwareMap!!.get(Servo::class.java, "RG")

        LG.position = 0.0
        RG.position = 0.1



        /* CONFIGURE STARTING POSE*/
        StartPose = if (ALLIANCE == BLUE && POSITION == CLOSE) {
            Pose2d(12.0, 72.0 - 17 / 2, Math.toRadians(0.0))
        } else if (ALLIANCE == BLUE && POSITION == FAR) {
            Pose2d(-35.0, 72.0 - 17 / 2, Math.toRadians(0.0))
        } else if (ALLIANCE == RED && POSITION == CLOSE) {
            Pose2d(-35.0, -72.0 + 17 / 2, Math.toRadians(90.0))
        } else if (ALLIANCE == RED && POSITION == FAR) {
            Pose2d(12.0, -72.0 + 17 / 2, Math.toRadians(90.0))
        }
        else {
            Pose2d(-35.0, 72.0 - 17 / 2, Math.toRadians(0.0))
        }
        DRIVE.poseEstimate = StartPose
        /* END CONFIGURE STARTING POSE */


        /* SEQUENCES */
        val start = DRIVE.trajectorySequenceBuilder(StartPose)
                .forward(42 - (17.0 / 2.0))
                .build()

        val left = DRIVE.trajectorySequenceBuilder(start.end())
                .turn(Math.toRadians(90.0))
                .addDisplacementMarker {
                    hardwareMap.get(Servo::class.java, "RG").position = TestVars.RGOpen
                }
                .waitSeconds(1.0)
                .addDisplacementMarker {
                    DRIVE.CRANE.power = 0.2
                }
                .waitSeconds(1.0)
                .addDisplacementMarker {
                    DRIVE.CRANE.power = 0.0
                    hardwareMap.get(Servo::class.java, "RG").position = TestVars.RGClose
                }
                .turn(Math.toRadians(-90.0))
                .build()

        val right = DRIVE.trajectorySequenceBuilder(start.end())
                .turn(Math.toRadians(-90.0))
                .addDisplacementMarker {
                    hardwareMap.get(Servo::class.java, "RG").position = TestVars.RGOpen
                }
                .waitSeconds(1.0)
                .addDisplacementMarker{
                    DRIVE.CRANE.power = 0.2
                }
                .waitSeconds(1.0)
                .addDisplacementMarker{
                    DRIVE.CRANE.power = 0.0
                    hardwareMap.get(Servo::class.java, "RG").position = TestVars.RGClose
                }
                .turn(Math.toRadians(90.0))
                .build()

        val center = DRIVE.trajectorySequenceBuilder(start.end())
                .addDisplacementMarker{
                    hardwareMap.get(Servo::class.java, "RG").position = TestVars.RGOpen
                }
                .waitSeconds(1.0)
                .addDisplacementMarker{
                    DRIVE.CRANE.power = 0.2
                    hardwareMap.get(Servo::class.java, "RG").position = TestVars.RGClose
                }
                .waitSeconds(0.5)
                .addDisplacementMarker{
                    DRIVE.CRANE.power = 0.0
                    hardwareMap.get(Servo::class.java, "RG").position = TestVars.RGClose
                }
                .build()

        val returnToStartPos = DRIVE.trajectorySequenceBuilder(center.end())
                .back(42 - (17.0 / 2.0))
                .build()


        /* END SEQUENCES */

        while (!isStarted) {
            telemetry.addData("Cube Location", CAMERA.cubeLocation)
            telemetry.update()
        }

        waitForStart();

        pathToFollow.add(start)

        when (CAMERA.cubeLocation) {
            "CENTER" -> {
                pathToFollow.add(center)
                Mult = 2
            }
            "LEFT" -> {
                pathToFollow.add(left)
                Mult = 1
            }
            "RIGHT" -> {
                pathToFollow.add(right)
                Mult = 3
            }
        }



        pathToFollow.add(returnToStartPos)
        val toBoard = if (POSITION == CLOSE) {
            val color = if (ALLIANCE == BLUE) {
                1
            } else {
                -1
            }
            DRIVE.trajectorySequenceBuilder(returnToStartPos.end())
                    .turn(Math.toRadians(color * 90.0))
                    .forward(24.0)
                    .turn(Math.toRadians(color * -90.0))
                    .forward(Mult * 6.0 + 12)
                    .turn(Math.toRadians(color * 90.0))
                    .build()
        } else {
            val color = if (ALLIANCE == BLUE) {
                1
            } else {
                -1
            }
            DRIVE.trajectorySequenceBuilder(returnToStartPos.end())
                    .forward(12 - 17.0 / 2.0)
                    .turn(Math.toRadians( color * 90.0))
                    .forward(3*24.0)
                    .turn(Math.toRadians(color * -90.0))
                    .forward((12 - 17.0 / 2.0)/2)
                    .forward(Mult * 6.0 + 12)
                    .turn(color * Math.toRadians(90.0))
                    .build()
        }
        pathToFollow.add(toBoard)

        val park = if (POSITION == CLOSE) {
            val color = if (ALLIANCE == BLUE) {
                1
            } else {
                -1
            }
            DRIVE.trajectorySequenceBuilder(toBoard.end())
                    .turn(Math.toRadians( color * -90.0))
                    .back(2 * 6.0 + 12)
                    .turn(Math.toRadians(color * 90.0))
                    .forward(24.0)
                    .build()
        } else {
            val color = if (ALLIANCE == BLUE) {
                1
            } else {
                -1
            }
            DRIVE.trajectorySequenceBuilder(toBoard.end())
                    .turn(Math.toRadians( color * -90.0))
                    .back(Mult * 6.0 + 12)
                    .turn(Math.toRadians(color * 90.0))
                    .forward(24.0)
                    .build()
        }

        telemetry.addData("Status", "Running");
        telemetry.update();


        while (!pathToFollow.isEmpty()) {
            DRIVE.followTrajectorySequence(pathToFollow.poll())
        }

        stop()
        /* FUNCTIONS TO PLACE ON BACKDROP */
        val forwardTen = DRIVE.trajectorySequenceBuilder(toBoard.end())
                .forward(15.0)
                .build()

        val backTen = DRIVE.trajectorySequenceBuilder(forwardTen.end())
                .back(3.0)
                .build()
        fun liftToPosition(){
            LIFT.power = 1.0
            sleep(700)
            LIFT.power = 0.05
            WRIST.position = TestVars.WristTop
            sleep(1500)
        }


        fun releasePixel() {
            LG.position = 0.1
            sleep(1000)
        }

        fun lowerArm() {
            LG.position = 0.0
            WRIST.position = TestVars.WristLevelPos
            sleep(1500)
            LIFT.power = -0.8
            sleep(700)
            LIFT.power = 0.0

        }
        /* END FUNCTIONS TO PLACE ON BACKDROP */
        liftToPosition()
        DRIVE.followTrajectorySequence(forwardTen)
        releasePixel()
        DRIVE.followTrajectorySequence(backTen)
        lowerArm()
        //DRIVE.followTrajectorySequence(park)
    }
}


@Autonomous(name = "Arm Test")
class armTest: LinearOpMode() {
    override fun runOpMode() {
        var LIFT: DcMotorEx
        var WRIST: Servo
        var LG: Servo
        var RG: Servo

        LIFT = hardwareMap!!.get(DcMotorEx::class.java, "LIFT")
        LIFT.direction = DcMotorSimple.Direction.REVERSE

        WRIST = hardwareMap!!.get(Servo::class.java, "WRIST")
        LG = hardwareMap!!.get(Servo::class.java, "LG")
        RG = hardwareMap!!.get(Servo::class.java, "RG")


        LG.position = 0.0
        RG.position = 0.1
        waitForStart()

        LIFT.power = 1.0
        sleep(700)
        LIFT.power = 0.05
        WRIST.position = TestVars.WristTop
        sleep(1500)
        LG.position = 0.1
        sleep(1000)
        LG.position = 0.0
        WRIST.position = TestVars.WristLevelPos
        sleep(1000)
        LIFT.power = -0.8
        sleep(700)

    }

}
@Autonomous(name = "RoadRunner Arm Test")
class RoadArmTest: LinearOpMode() {
    override fun runOpMode() {
        var LIFT: DcMotorEx
        var WRIST: Servo
        var LG: Servo
        var RG: Servo
        var StartPose: Pose2d
        val DRIVE = SampleMecanumDrive(hardwareMap);
        StartPose = Pose2d(0.0,0.0,Math.toRadians(0.0))

        var pathToFollow: Queue<TrajectorySequence> = LinkedList()

        LIFT = hardwareMap!!.get(DcMotorEx::class.java, "LIFT")
        LIFT.direction = DcMotorSimple.Direction.REVERSE

        WRIST = hardwareMap!!.get(Servo::class.java, "WRIST")
        LG = hardwareMap!!.get(Servo::class.java, "LG")
        RG = hardwareMap!!.get(Servo::class.java, "RG")

        LG.position = 0.0
        RG.position = 0.1

        val backTen = DRIVE.trajectorySequenceBuilder(StartPose)
                .back(10.0)
                .build()
        fun liftToPosition(){
            LIFT.power = 1.0
            sleep(700)
            LIFT.power = 0.05
            WRIST.position = TestVars.WristTop
            sleep(1500)
        }

        val forwardTen = DRIVE.trajectorySequenceBuilder(StartPose)
                .forward(10.0)
                .build()

        fun releasePixel() {
            LG.position = 0.1
            sleep(1000)
        }

        fun lowerArm() {
            LG.position = 0.0
            WRIST.position = TestVars.WristLevelPos
            sleep(1500)
            LIFT.power = -0.8
            sleep(700)
            LIFT.power = 0.0

        }
        val sequence = DRIVE.trajectorySequenceBuilder(StartPose)
                .back(10.0)
                .addDisplacementMarker{
                    LIFT.power = 1.0
                    sleep(700)
                }
                .addDisplacementMarker{
                    LIFT.power = 0.05
                    WRIST.position = TestVars.WristTop
                    sleep(1500)
                }
                .forward(10.0)
                .addDisplacementMarker{
                    LG.position = 0.1
                    sleep(1000)
                }
                .forward(-10.0)
                .addDisplacementMarker{
                    LG.position = 0.0
                    WRIST.position = TestVars.WristLevelPos
                    sleep(1000)
                }
                .addDisplacementMarker{
                    LIFT.power = -0.8
                    sleep(800)
                }
                .addDisplacementMarker{
                    LIFT.power = 0.0
                }
                .build()

        waitForStart()
        DRIVE.followTrajectorySequence(backTen)
        liftToPosition()
        DRIVE.followTrajectorySequence(forwardTen)
        releasePixel()
        DRIVE.followTrajectorySequence(backTen)
        lowerArm()

    }
}