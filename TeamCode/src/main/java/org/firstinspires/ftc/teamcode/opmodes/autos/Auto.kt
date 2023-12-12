package org.firstinspires.ftc.teamcode.opmodes.autos

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
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
        val ALLIANCE = BLUE
        val POSITION = FAR

        val CAMERA = Camera(hardwareMap, telemetry, ALLIANCE);

        val DRIVE = SampleMecanumDrive(hardwareMap);
        var StartPose: Pose2d
        var pathToFollow: Queue<TrajectorySequence> = LinkedList()

        var Mult: Int = 2



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
                .forward(48 - (17.0 / 2.0))
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
                }
                .waitSeconds(1.0)
                .addDisplacementMarker{
                    DRIVE.CRANE.power = 0.0
                    hardwareMap.get(Servo::class.java, "RG").position = TestVars.RGClose
                }
                .build()

        val returnToStartPos = DRIVE.trajectorySequenceBuilder(center.end())
                .back(48 - (17.0 / 2.0))
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
        pathToFollow.add(park)

        telemetry.addData("Status", "Running");
        telemetry.update();


        while (!pathToFollow.isEmpty()) {
            DRIVE.followTrajectorySequence(pathToFollow.poll())
        }
    }
}
