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
    LEFT, RIGHT
}

@Autonomous(name = "AUTONOMOUS")
class Auto: LinearOpMode() {

    override fun runOpMode() {
        val ALLIANCE = BLUE
        val POSITION = RIGHT

        val CAMERA = Camera(hardwareMap, telemetry, ALLIANCE);

        val DRIVE = SampleMecanumDrive(hardwareMap);
        var StartPose: Pose2d = Pose2d(-36.0, 72.0, Math.toRadians(0.0))
        var pathToFollow: Queue<TrajectorySequence> = LinkedList()



//        /* CONFIGURE STARTING POSE*/
       /* if (ALLIANCE == BLUE && POSITION == LEFT) {
            StartPose = Pose2d(12.0, 72.0 - 17 / 2, Math.toRadians(90.0))
        } else if (ALLIANCE == BLUE && POSITION == RIGHT) {
            StartPose = Pose2d(-35.0, 72.0 - 17 / 2, Math.toRadians(0.0))
        } else if (ALLIANCE == RED && POSITION == LEFT) {
            StartPose = Pose2d(-35.0, -72.0 + 17 / 2, Math.toRadians(180.0))
        } else if (ALLIANCE == RED && POSITION == RIGHT) {
            StartPose = Pose2d(12.0, -72.0 + 17 / 2, Math.toRadians(180.0))
        } */
        StartPose = Pose2d(12.0, 72.0 - 17/2, Math.toRadians(90.0))
        /* END CONFIGURE STARTING POSE */


        /* SEQUENCES */
        val start = DRIVE.trajectorySequenceBuilder(StartPose)
                .forward(48 - (17.0 / 2.0))
                .build()

        val left = DRIVE.trajectorySequenceBuilder(DRIVE.poseEstimate)
                .turn(Math.toRadians(90.0))
                .addDisplacementMarker {
                    toggleLeftClaw()
                }
                .waitSeconds(0.5)
                .addDisplacementMarker {
                    DRIVE.CRANE.power = 0.2
                }
                .waitSeconds(0.5)
                .addDisplacementMarker {
                    DRIVE.CRANE.power = 0.0
                    toggleLeftClaw()
                }
                .turn(Math.toRadians(-90.0))
                .build()

        val right = DRIVE.trajectorySequenceBuilder(DRIVE.poseEstimate)
                .turn(Math.toRadians(-90.0))
                .addDisplacementMarker {
                    toggleRightClaw()
                }
                .waitSeconds(.5)
                .addDisplacementMarker{
                    DRIVE.CRANE.power = 0.2
                }
                .waitSeconds(0.5)
                .addDisplacementMarker{
                    DRIVE.CRANE.power = 0.0
                    toggleRightClaw()
                }
                .turn(Math.toRadians(90.0))
                .build()

        val center = DRIVE.trajectorySequenceBuilder(DRIVE.poseEstimate)
                .addDisplacementMarker{
                    toggleLeftClaw()
                }
                .waitSeconds(0.5)
                .addDisplacementMarker{
                    DRIVE.CRANE.power = 0.2
                }
                .waitSeconds(0.5)
                .addDisplacementMarker{
                    DRIVE.CRANE.power = 0.0
                    toggleLeftClaw()
                }
                .build()

        val returnToStartPos = DRIVE.trajectorySequenceBuilder(DRIVE.poseEstimate)
                //.lineTo(StartPose.vec())
                //.build()
                .back(48 - (17.0 / 2.0))
                .build()

        val park = DRIVE.trajectorySequenceBuilder(DRIVE.poseEstimate)


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
            }
            "LEFT" -> {
                pathToFollow.add(left)
            }
            "RIGHT" -> {
                pathToFollow.add(right)
            }
        }

        pathToFollow.add(returnToStartPos)

        telemetry.addData("Status", "Running");
        telemetry.update();


        while (!pathToFollow.isEmpty()) {
            DRIVE.followTrajectorySequence(pathToFollow.poll())
        }
    }

    fun toggleLeftClaw() {
        var LG: Servo
        LG = hardwareMap!!.get(Servo::class.java, "LG")
        if (LG.position == TestVars.LGClose) {
            LG.position = TestVars.LGOpen
        }
        else {
            LG.position = TestVars.LGClose
        }
    }

    fun toggleRightClaw() {
        var RG: Servo
        RG = hardwareMap!!.get(Servo::class.java, "RG")
        if (RG.position == TestVars.RGClose) {
            RG.position = TestVars.RGOpen
        }
        else {
            RG.position = TestVars.RGClose
        }
    }
}
